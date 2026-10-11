#!/usr/bin/env python3
"""Isolated ROS smoke test: camera -> VPR -> waypoints -> odom Path, no controller.

Starts its own localhost ROS master and cleans up all child nodes. Requires ROS
environment, prepared weights, and three numbered .jpg topomap images.
"""
import argparse
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wp-model", choices=["gnm_large", "vint", "nomad"], required=True)
    parser.add_argument("--pr-model", choices=["cosplace", "megaloc", "salad", "anyloc"], default="cosplace")
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--port", type=int, default=11329)
    args = parser.parse_args()
    # Refuse to attach to any existing master (especially a vehicle's master).
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", args.port))
    os.environ.update(ROS_MASTER_URI=f"http://127.0.0.1:{args.port}", ROS_IP="127.0.0.1")
    os.environ.pop("ROS_HOSTNAME", None)
    children = []
    with tempfile.TemporaryDirectory(prefix="placenav-wp-ros-") as directory:
        folder = Path(directory)
        route = folder / "test_route"
        route.mkdir()
        for idx in range(3):
            shutil.copyfile(args.images / f"{idx}.jpg", route / f"{idx}.jpg")
        log_path = folder / "nodes.log"
        with log_path.open("w") as log:
            try:
                children.append(subprocess.Popen(["roscore", "-p", str(args.port)],
                                                  stdout=log, stderr=log, start_new_session=True))
                master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
                for _ in range(100):
                    try:
                        master.getPid("/waypoint_smoke")
                        break
                    except OSError:
                        time.sleep(0.1)
                else:
                    raise RuntimeError("Test ROS master did not start")
                import rospy
                import cv2
                import cv_bridge
                import tf2_ros
                from geometry_msgs.msg import TransformStamped
                from sensor_msgs.msg import Image
                from nav_msgs.msg import Path as RosPath
                from placenav_viz_msgs.msg import Viz
                rospy.init_node("waypoint_smoke", disable_signals=True)
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = "odom", "base_link"
                tf.transform.rotation.w = 1
                broadcaster = tf2_ros.StaticTransformBroadcaster()
                broadcaster.sendTransform(tf)
                paths, viz = [], []
                rospy.Subscriber("/placenav/gnm_path", RosPath, paths.append)
                rospy.Subscriber("/toponav/viz_info", Viz, viz.append)
                publisher = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
                bridge = cv_bridge.CvBridge()
                rgb = cv2.imread(str(args.images / "0.jpg"))
                root = Path(__file__).resolve().parent
                # Start the public wrapper to test its model selection and adapter.
                children.append(subprocess.Popen([
                    "bash", str(root / "navigate_whill.sh"), "--topomap-dir", "test_route",
                    "--topomap-base-dir", str(folder), "--wp-model", args.wp_model,
                    "--pr-model", args.pr_model, "--filter-mode", "bayesian", "--no-visualization"],
                    stdout=log, stderr=log, start_new_session=True))
                deadline = time.monotonic() + 90
                while time.monotonic() < deadline and not paths:
                    if children[-1].poll() is not None:
                        raise RuntimeError("Navigation exited before producing a Path")
                    msg = bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
                    msg.header.stamp = rospy.Time.now()
                    publisher.publish(msg)
                    time.sleep(0.2)
                count = 8 if args.wp_model == "nomad" else 5
                assert paths and viz, "No path received within timeout"
                assert len(viz[-1].waypoints) == count
                assert len(paths[-1].poses) == count + 1
                assert paths[-1].header.frame_id == "odom"
                print(f"PASS ROS {args.pr_model}+{args.wp_model}: Viz={count}, odom Path={count+1} poses")
            except Exception:
                log.flush()
                print(log_path.read_text())
                raise
            finally:
                for child in reversed(children):
                    if child.poll() is None:
                        os.killpg(child.pid, signal.SIGTERM)
                        try:
                            child.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            os.killpg(child.pid, signal.SIGKILL)
                            child.wait()


if __name__ == "__main__":
    main()
