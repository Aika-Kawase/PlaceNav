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
    parser.add_argument("--mode", choices=["state_machine", "latest"], required=True)
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
            config = dict(reference_mode=args.mode, teb_local_plan_topic="/test_seed", path_timeout=1., path_topic="/test_reference",
                          cmd_vel_topic="/nomad_test/cmd_vel", controller_frequency=5.,
                          footprint=[[-.15,-.3],[.85,-.3],[.85,.3],[-.15,.3]],
                          local_costmap=dict(global_frame="odom", robot_base_frame="base_link",
                                             rolling_window=True, width=6., height=6.,
                                             resolution=.05, plugins=[dict(name="static", type="costmap_2d::StaticLayer")],
                                             static=dict(map_topic="/test_map"), static_map=False,
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
            commands, plans = [], []
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
            assert reports, "No diagnostics published"
            # Nearby obstacle triggers the bridge's corridor test, while leaving
            # the straight path physically traversable for the small test robot.
            grid.data[72 * 120 + 80] = 100  # x=1.025, y=.625 in odom
            grid.header.stamp = rospy.Time.now()
            maps.publish(grid)
            for _ in range(25):
                feed(seed=True, target_y=.3)
                feed()
                time.sleep(.1)
            values = {v.key:v.value for v in reports[-1].status[0].values}
            assert int(values["cells_near_path_points"]) > 0, values
            expected = "AVOIDING" if args.mode == "state_machine" else "TRACKING"
            assert values["state"] == expected, values
            # Moving the input slightly away still leaves the obstacle inside
            # the .8 m corridor. Only latest mode should adopt this new input.
            for _ in range(15):
                feed(target_y=-.25)
                time.sleep(.1)
            goal_y = plans[-1].poses[-1].pose.position.y
            assert abs(goal_y - (.3 if args.mode == "state_machine" else -.25)) < .001, goal_y
            print("PASS reference switching: " + args.mode, flush=True)
            time.sleep(1.5)
            commands.clear()
            time.sleep(.6)
            assert commands and not moving(), "Expired reference did not stop"
            assert reports[-1].status[0].message == "INPUT_TIMEOUT"
            print("PASS expired reference stops with diagnostic", flush=True)
            # A near-zero reference is reached; a new equivalent reference must
            # be processed again, rather than silently remaining inactive.
            maps.publish(OccupancyGrid(header=grid.header, info=grid.info, data=[0]*14400))
            for _ in range(20):
                feed(end=.01)
                time.sleep(.1)
            assert reports[-1].status[0].message == "GOAL_REACHED", reports[-1]
            for _ in range(10):
                feed()
                time.sleep(.1)
            assert reports[-1].status[0].message == "COMMAND_VALID", reports[-1]
            print("PASS reached goal and new-path restart", flush=True)
            # Fully occupied map: TEB failure must not be reported as arrival.
            grid.data = [100]*14400
            grid.header.stamp = rospy.Time.now()
            maps.publish(grid)
            for _ in range(15):
                feed()
                time.sleep(.1)
            assert reports[-1].status[0].message == "TEB_FAILED", reports[-1]
            values = {v.key:v.value for v in reports[-1].status[0].values}
            assert int(values["teb_outcome"]) != 0
            assert commands[-1].linear.x == 0. and commands[-1].angular.z == 0.
            print("PASS blocked map gives TEB failure and zero command", flush=True)
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
