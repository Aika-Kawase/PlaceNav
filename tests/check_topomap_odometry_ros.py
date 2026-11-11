#!/usr/bin/env python3
"""Run the real topomap wrapper on an isolated ROS master; no robot publishers."""
import argparse
import json
import math
import os
from pathlib import Path
import pickle
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=11339)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", args.port))
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_MASTER_URI=f"http://127.0.0.1:{args.port}", ROS_IP="127.0.0.1")
    os.environ.pop("ROS_HOSTNAME", None)
    children = []
    root = Path(__file__).resolve().parents[1]
    log = (args.output / "synthetic_ros.log").open("w")
    try:
        master_process = subprocess.Popen(["roscore", "-p", str(args.port)],
                                          stdout=log, stderr=log, start_new_session=True)
        children.append(master_process)
        master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
        for _ in range(100):
            try:
                master.getPid("/topomap_test")
                break
            except OSError:
                time.sleep(.1)
        else:
            raise RuntimeError("Test master did not start")
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image
        rospy.init_node("topomap_test", disable_signals=True)
        camera = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=10)
        wheel = rospy.Publisher("/odom", Odometry, queue_size=100)
        alternate = rospy.Publisher("/fixture_vo/odom", Odometry, queue_size=100)

        def wait_until(predicate, message, timeout=8):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if predicate():
                    return
                time.sleep(.02)
            raise AssertionError(message)

        def start(name, options, publisher=wheel, positional=False):
            # ROS can briefly retain the previous process's TCP connections.
            wait_until(lambda: not camera.get_num_connections() and
                       not publisher.get_num_connections(),
                       "Previous generator subscriptions still connected")
            command = ["bash", str(root / "src/create_topomap_whill.sh"), name]
            if positional:
                command.append("0.1")
            command += ["--topomap_directory", str(args.output)] + options
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            children.append(process)
            wait_until(lambda: camera.get_num_connections() and publisher.get_num_connections(),
                       "Generator subscriptions not ready")
            return process, args.output / name, command

        def stop(process):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=5)
            assert process.returncode == 0, "Generator failed"

        def odom(publisher, stamp, pose, frame="odom"):
            msg = Odometry()
            msg.header.stamp = rospy.Time.from_sec(stamp)
            msg.header.frame_id = frame
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
            msg.pose.pose.orientation.z = math.sin(pose[2] / 2)
            msg.pose.pose.orientation.w = math.cos(pose[2] / 2)
            publisher.publish(msg)

        def image(stamp, value):
            msg = Image()
            msg.header.stamp = rospy.Time.from_sec(stamp)
            msg.height, msg.width, msg.step = 8, 8, 24
            msg.encoding = "rgb8"
            msg.data = bytes([value, 40, 60]) * 64
            camera.publish(msg)

        def bracket(publisher, stamp, pose, value, frame="odom"):
            x, y, yaw = pose
            odom(publisher, stamp, (x-.2, y-.1, yaw), frame)
            time.sleep(.03)
            image(stamp+.05, value)
            time.sleep(.03)  # The matching future odometry arrives after the image.
            odom(publisher, stamp+.1, (x+.2, y+.1, yaw), frame)

        def node_count(folder):
            path = folder / "node_odometry.json"
            return len(json.loads(path.read_text())["nodes"]) if path.exists() else 0

        # Check skipped observations, image-time interpolation, turns and node deltas.
        process, folder, command = start("odometry", [
            "--map-mode", "odometry", "--straight-distance", "2.0"])
        samples = [(0, 0, 0), (.5, 0, 0), (2, 0, 0),
                   (2.2, 0, math.pi/2), (2.2, 2.1, math.pi/2)]
        expected_counts = [1, 1, 2, 3, 4]
        for i, (pose, count) in enumerate(zip(samples, expected_counts)):
            bracket(wheel, 100+i, pose, 20+20*i)
            time.sleep(.2)
            wait_until(lambda: node_count(folder) == count, "Wrong saved node count")
        live = json.loads((folder / "node_odometry.json").read_text())
        assert len(list(folder.glob("*.jpg"))) == 4
        expected_poses = [samples[i] for i in (0, 2, 3, 4)]
        for node, expected, index in zip(live["nodes"], expected_poses, (0, 2, 3, 4)):
            assert node["image"] == str(node["id"]) + ".jpg"
            assert abs(node["stamp"]["secs"] + node["stamp"]["nsecs"]/1e9 -
                       (100+index+.05)) < 1e-8
            for key, value in zip(("x", "y", "yaw"), expected):
                assert abs(node["pose"][key] - value) < 1e-8, (node, expected)
        for edge, expected in zip(live["edges"], [(2, 0, 0), (.2, 0, math.pi/2), (2.1, 0, 0)]):
            for key, value in zip(("dx", "dy", "dyaw"), expected):
                assert abs(edge[key] - value) < 1e-8, edge
        # JSON is already durable while the generator is still alive.
        assert process.poll() is None
        stop(process)
        with (folder / "traj_data.pkl").open("rb") as stream:
            trajectory = pickle.load(stream)
        assert len(trajectory["position"]) == 4
        assert abs(trajectory["relative_position"][1][0]-2) < 1e-8
        original = {p.name: p.read_bytes() for p in folder.iterdir()}
        rerun = subprocess.run(command, stdout=log, stderr=log, timeout=8)
        assert rerun.returncode != 0
        assert original == {p.name: p.read_bytes() for p in folder.iterdir()}
        print("PASS image-time pose, skipped-node deltas, turns, live JSON, existing-map protection", flush=True)

        # The original two-positional-argument command still uses /odom and legacy output.
        process, folder, _ = start("legacy", ["--sampling-mode", "time"], positional=True)
        for i in range(3):
            odom(wheel, 200+i, (2*i, 0, 0))
            time.sleep(.03)
            image(200+i+.05, 80+20*i)
            wait_until(lambda: len(list(folder.glob("*.jpg"))) == i+1,
                       "Legacy image missing")
            time.sleep(.12)
        stop(process)
        assert not (folder / "node_odometry.json").exists()
        with (folder / "traj_data.pkl").open("rb") as stream:
            trajectory = pickle.load(stream)
        assert set(trajectory) == {"position", "yaw", "relative_position", "relative_yaw"}
        assert [float(p[0]) for p in trajectory["position"]] == [0, 2, 4]
        print("PASS legacy CLI, default wheel topic, legacy pkl/image format", flush=True)

        # A VO-style topic can replace the default without changing the algorithm.
        process, folder, _ = start("alternate", [
            "--map-mode", "odometry", "--odom-topic", "/fixture_vo/odom",
            "--straight-distance", "2.0"], publisher=alternate)
        # A wide bracket must not create an image or JSON node.
        odom(alternate, 300, (0, 0, 0), "vo_odom")
        odom(alternate, 301, (20, 0, 0), "vo_odom")
        time.sleep(.03)
        image(300.5, 120)
        time.sleep(.2)
        assert not list(folder.glob("*.jpg"))
        for i in range(2):
            odom(wheel, 400+i, (999, 999, 0))  # Wrong source must be ignored.
            bracket(alternate, 302+i, (2.1*i, 0, 0), 140+20*i, "vo_odom")
            wait_until(lambda: node_count(folder) == i+1, "Alternate odometry not used")
            time.sleep(.12)
        data = json.loads((folder / "node_odometry.json").read_text())
        assert data["odom_topic"] == "/fixture_vo/odom" and data["frame_id"] == "vo_odom"
        assert abs(data["edges"][0]["dx"] - 2.1) < 1e-8
        original = (folder / "node_odometry.json").read_bytes()
        odom(alternate, 305, (0, 0, 0), "different_frame")
        process.wait(timeout=5)
        assert process.returncode != 0
        assert (folder / "node_odometry.json").read_bytes() == original
        print("PASS alternate topic, missing-data rejection, coordinate-frame change protection", flush=True)
        process, folder, _ = start("time_reset", [
            "--map-mode", "odometry", "--straight-distance", "2.0"])
        bracket(wheel, 500, (0, 0, 0), 180)
        wait_until(lambda: node_count(folder) == 1, "Reset fixture initial node missing")
        original = (folder / "node_odometry.json").read_bytes()
        odom(wheel, 499, (10, 10, 0))
        process.wait(timeout=5)
        assert process.returncode != 0
        assert (folder / "node_odometry.json").read_bytes() == original
        print("PASS odometry timestamp reset stops without changing saved nodes", flush=True)

        # Long stops are fine; a far-away image after lost input must not become
        # a normal edge. Keep the JPEG, JSON and pkl prefix intact on failure.
        process, folder, _ = start("missing_travel", [
            "--map-mode", "odometry", "--straight-distance", "2.0"])
        bracket(wheel, 600, (0, 0, 0), 180)
        wait_until(lambda: node_count(folder) == 1, "Gap fixture initial node missing")
        bracket(wheel, 660, (0, 0, 0), 190)
        time.sleep(.2)
        assert process.poll() is None and node_count(folder) == 1
        original = (folder / "node_odometry.json").read_bytes()
        bracket(wheel, 727, (37.447035, 44.140099, 1.769024), 200)
        process.wait(timeout=5)
        assert process.returncode != 0
        assert (folder / "node_odometry.json").read_bytes() == original
        assert len(list(folder.glob("*.jpg"))) == 1
        with (folder / "traj_data.pkl").open("rb") as stream:
            assert len(pickle.load(stream)["position"]) == 1
        print("PASS stationary pause allowed; 58m recovery rejected before saving", flush=True)

        # Every available 2m observation is kept, including after a long stop.
        process, folder, _ = start("regular_distance", [
            "--map-mode", "odometry", "--straight-distance", "2.0"])
        for i in range(8):
            bracket(wheel, 800+i, (2.01*i, 0, 0), 30+20*i)
            wait_until(lambda: node_count(folder) == i+1, "Regular 2m node missing")
            time.sleep(.12)
        stop(process)
        data = json.loads((folder / "node_odometry.json").read_text())
        assert data["sampling"]["max_node_distance"] == 4.0
        assert all(abs(edge["distance"]-2.01) < 1e-8 for edge in data["edges"])

        process, folder, _ = start("explicit_distance_limit", [
            "--map-mode", "odometry", "--straight-distance", "2.0",
            "--max-node-distance", "2.2"])
        bracket(wheel, 900, (0, 0, 0), 180)
        wait_until(lambda: node_count(folder) == 1, "Limit fixture initial node missing")
        bracket(wheel, 901, (2.3, 0, 0), 200)
        process.wait(timeout=5)
        assert process.returncode != 0 and node_count(folder) == 1
        assert len(list(folder.glob("*.jpg"))) == 1
        print("PASS regular 2m observations retained; explicit distance limit applied", flush=True)
        (args.output / "synthetic_ros.json").write_text(json.dumps({
            "image_time_interpolation": True, "saved_node_deltas": True,
            "heading_rotation": True, "live_json": True, "existing_map_preserved": True,
            "legacy_cli_and_format": True, "alternate_odom_topic": True,
            "missing_odom_rejected": True, "frame_change_stops": True,
            "time_reset_stops": True,
            "stationary_pause_allowed": True, "long_edge_rejected_before_save": True,
            "regular_2m_nodes_retained": True, "explicit_distance_limit": True,
        }, indent=2) + "\n")
    except Exception:
        log.flush()
        print((args.output / "synthetic_ros.log").read_text()[-12000:], flush=True)
        raise
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        log.close()


if __name__ == "__main__":
    main()
