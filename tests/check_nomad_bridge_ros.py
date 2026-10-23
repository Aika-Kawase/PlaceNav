#!/usr/bin/env python3
"""Isolated bridge invalidation/expiry integration test. Never use vehicle master."""
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
            from nav_msgs.msg import Path as RosPath, Odometry
            rospy.init_node("nomad_bridge_test", disable_signals=True)
            config = dict(path_timeout=1., path_topic="/test_reference",
                          cmd_vel_topic="/nomad_test/cmd_vel", controller_frequency=5.,
                          footprint=[[-.15,-.3],[.85,-.3],[.85,.3],[-.15,.3]],
                          local_costmap=dict(global_frame="odom", robot_base_frame="base_link",
                                             rolling_window=True, width=6., height=6.,
                                             resolution=.05, plugins=[], static_map=False,
                                             track_unknown_space=False),
                          TebLocalPlannerROS=dict(odom_topic="/test_odom",
                                                 enable_homotopy_class_planning=False,
                                                 max_vel_x=.4, max_vel_theta=.4,
                                                 xy_goal_tolerance=.1))
            rospy.set_param("/placenav_teb_bridge", config)
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = "odom", "base_link"
            tf.transform.rotation.w = 1.
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            broadcaster.sendTransform(tf)
            commands, plans = [], []
            rospy.Subscriber("/nomad_test/cmd_vel", Twist, commands.append)
            rospy.Subscriber("/placenav_teb_bridge/active_global_path", RosPath, plans.append)
            pub = rospy.Publisher("/test_reference", RosPath, queue_size=1)
            odom_pub = rospy.Publisher("/test_odom", Odometry, queue_size=1)
            children.append(subprocess.Popen([str(args.bridge), "__name:=placenav_teb_bridge"],
                                              stdout=log, stderr=log, start_new_session=True))

            def feed(empty=False):
                path = RosPath()
                path.header.frame_id, path.header.stamp = "odom", rospy.Time.now()
                if not empty:
                    for x in [0., .3, .6, .9, 1.2, 1.5, 1.8]:
                        p = PoseStamped()
                        p.header = path.header
                        p.pose.position.x, p.pose.orientation.w = x, 1.
                        path.poses.append(p)
                pub.publish(path)
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id = "odom", "base_link"
                odom.header.stamp = rospy.Time.now()
                odom.pose.pose.orientation.w = 1.
                odom_pub.publish(odom)

            def moving():
                return any(abs(c.linear.x)+abs(c.angular.z) > 1e-6 for c in commands)

            for phase in range(2):
                commands.clear()
                deadline = time.monotonic()+15
                while time.monotonic() < deadline and not moving():
                    feed()
                    time.sleep(.1)
                assert moving(), "No valid speed command on free straight path"
                print("PASS valid path generates command", flush=True)
                if phase == 0:
                    for _ in range(10):
                        feed(empty=True)
                        time.sleep(.1)
                    assert plans and not plans[-1].poses, "Empty selection retained active plan"
                    commands.clear()
                    for _ in range(5):
                        feed(empty=True)
                        time.sleep(.1)
                    assert commands and not moving(), "Empty selection did not hold zero"
                    print("PASS empty selection clears plan and holds zero", flush=True)
            time.sleep(1.5)
            commands.clear()
            time.sleep(.6)
            assert commands and not moving(), "Expired reference did not stop"
            print("PASS expired reference stops", flush=True)
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
