#!/usr/bin/env python3
"""Isolated reference-mode and stop-diagnostics integration test (no robot)."""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", required=True, type=Path)
    parser.add_argument("--port", type=int, default=11340)
    parser.add_argument("--mode", choices=["latest"], default="latest")
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", args.port))
    os.environ.update(ROS_MASTER_URI=f"http://127.0.0.1:{args.port}", ROS_IP="127.0.0.1")
    os.environ.pop("ROS_HOSTNAME", None)
    children = []
    with tempfile.TemporaryFile(mode="w+") as log:
        try:
            children.append(subprocess.Popen(["roscore", "-p", str(args.port)],
                                              stdout=log, stderr=log, start_new_session=True))
            master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
            for _ in range(100):
                try:
                    master.getPid("/nomad_bridge_test")
                    break
                except OSError:
                    time.sleep(.1)
            import rospy
            import tf2_ros
            from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
            from nav_msgs.msg import Path as RosPath, Odometry, OccupancyGrid
            from diagnostic_msgs.msg import DiagnosticArray
            rospy.init_node("nomad_bridge_test", disable_signals=True)
            config = dict(retain_safe_path=True, retention_max_duration=3., reference_mode=args.mode, teb_local_plan_topic="/placenav_teb_bridge/TebLocalPlannerROS/local_plan", path_timeout=1., path_topic="/test_reference",
                          cmd_vel_topic="/nomad_test/cmd_vel", controller_frequency=5.,
                          footprint=[[-.15,-.3],[.85,-.3],[.85,.3],[-.15,.3]],
                          local_costmap=dict(global_frame="odom", robot_base_frame="base_link",
                                             rolling_window=True, width=6., height=6.,
                                             resolution=.05, plugins=[dict(name="static", type="costmap_2d::StaticLayer")],
                                             static=dict(map_topic="/test_map"), static_map=False,
                                             track_unknown_space=False),
                          TebLocalPlannerROS=dict(odom_topic="/test_odom",
                                                 enable_homotopy_class_planning=False,
                                                 max_vel_x=.4, max_vel_theta=.4, min_obstacle_dist=.15,
                                                 xy_goal_tolerance=.1))
            rospy.set_param("/placenav_teb_bridge", config)
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = "odom", "base_link"
            tf.transform.rotation.w = 1.
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            broadcaster.sendTransform(tf)
            maps = rospy.Publisher("/test_map", OccupancyGrid, queue_size=1, latch=True)
            grid = OccupancyGrid()
            grid.header.frame_id = "odom"
            grid.info.resolution = .05
            grid.info.width = grid.info.height = 120
            grid.info.origin.position.x = grid.info.origin.position.y = -3.
            grid.info.origin.orientation.w = 1.
            grid.data = [0] * 14400
            maps.publish(grid)
            reports = []
            rospy.Subscriber("/placenav_teb_bridge/diagnostics", DiagnosticArray, reports.append)
            seed_pub = rospy.Publisher("/test_seed", RosPath, queue_size=1)
            commands, plans, held = [], [], []
            rospy.Subscriber("/placenav_teb_bridge/retained_safe_path", RosPath, held.append)
            rospy.Subscriber("/nomad_test/cmd_vel", Twist, commands.append)
            rospy.Subscriber("/placenav_teb_bridge/active_global_path", RosPath, plans.append)
            pub = rospy.Publisher("/test_reference", RosPath, queue_size=1)
            odom_pub = rospy.Publisher("/test_odom", Odometry, queue_size=1)
            children.append(subprocess.Popen([str(args.bridge), "__name:=placenav_teb_bridge"],
                                              stdout=log, stderr=log, start_new_session=True))

            def feed(empty=False, target_y=0., seed=False, end=1.8):
                path = RosPath()
                path.header.frame_id, path.header.stamp = "odom", rospy.Time.now()
                if not empty:
                    for step in range(7):
                        x = end * step / 6.
                        p = PoseStamped()
                        p.header = path.header
                        p.pose.position.x, p.pose.orientation.w = x, 1.
                        p.pose.position.y = target_y * step / 6.
                        path.poses.append(p)
                (seed_pub if seed else pub).publish(path)
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id = "odom", "base_link"
                odom.header.stamp = rospy.Time.now()
                odom.pose.pose.orientation.w = 1.
                odom_pub.publish(odom)

            def moving():
                return any(abs(c.linear.x)+abs(c.angular.z) > 1e-6 for c in commands)

            deadline=time.monotonic()+20
            while time.monotonic()<deadline and not moving():
                feed();time.sleep(.1)
            assert moving(), "No retained-path command"
            assert held and held[-1].poses, "No retained path published"
            print("PASS safe plan adopted and followed",flush=True)
            original=held[-1].header.stamp
            count=len(held)
            commands.clear()
            for i in range(10):
                feed(target_y=.03 if i%2 else -.03);time.sleep(.1)
            assert len(held)==count, "Safe plan was replaced by minor input updates"
            assert moving(), "Retained path stopped on minor input updates"
            print("PASS same safe trajectory survives reference updates",flush=True)
            # A materially new goal must release the retained trajectory.
            mark=len(held)
            for _ in range(10):feed(target_y=1.);time.sleep(.1)
            assert any(not p.poses for p in held[mark:]), "New goal did not release retained path"
            print("PASS changed goal releases retained trajectory",flush=True)
            commands.clear();deadline=time.monotonic()+15
            while time.monotonic()<deadline and not moving():feed();time.sleep(.1)
            # Obstacle inside current footprint (including interior, not only edges).
            grid.data[60*120+68]=100
            grid.header.stamp=rospy.Time.now();maps.publish(grid)
            for _ in range(12):feed();time.sleep(.1)
            commands.clear()
            for _ in range(5):feed();time.sleep(.1)
            assert commands and not moving(), "Obstacle did not stop retained follower"
            assert held and not held[-1].poses, "Unsafe retained path not cleared"
            print("PASS new obstacle clears path and holds zero",flush=True)
            grid.data=[0]*14400;grid.header.stamp=rospy.Time.now();maps.publish(grid)
            commands.clear();deadline=time.monotonic()+15
            while time.monotonic()<deadline and not moving():feed();time.sleep(.1)
            assert moving(), "Did not resume after obstacle removed"
            print("PASS safe replanning resumes",flush=True)
            for _ in range(5):feed(empty=True);time.sleep(.1)
            commands.clear()
            for _ in range(5):feed(empty=True);time.sleep(.1)
            assert commands and not moving(), "Empty input did not stop retained follower"
            print("PASS empty input invalidates retained path",flush=True)
            commands.clear();deadline=time.monotonic()+15
            while time.monotonic()<deadline and not moving():feed();time.sleep(.1)
            assert moving()
            time.sleep(1.5);commands.clear();time.sleep(.5)
            assert commands and not moving(), "Input timeout did not stop retained follower"
            print("PASS input timeout invalidates retained path",flush=True)
        except Exception:
            log.seek(0)
            print(log.read())
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
