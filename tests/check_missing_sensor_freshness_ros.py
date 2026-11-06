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
    parser.add_argument("--mode", choices=["avoidance_commit", "latest"], default="avoidance_commit")
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
            from std_msgs.msg import String
            rospy.init_node("nomad_bridge_test", disable_signals=True)
            config = dict(retain_safe_path=True, retention_max_duration=.3, reference_mode=args.mode, teb_local_plan_topic="/placenav_teb_bridge/TebLocalPlannerROS/local_plan", path_timeout=1., path_topic="/test_reference",
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
            # Keep both a map and a configured sensor whose stream is absent.
            # This layer becomes not-current after 0.2s, without marking obstacles.
            config["local_costmap"]["plugins"].append(dict(name="missing_sensor",type="spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"))
            config["local_costmap"]["missing_sensor"]=dict(enabled=True,voxel_decay=8.,voxel_size=.05,
                track_unknown_space=False,observation_sources="cloud",cloud=dict(data_type="PointCloud2",
                topic="/test_missing_points",marking=True,clearing=False,expected_update_rate=.2))
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
            states=[]
            rospy.Subscriber("/placenav_teb_bridge/navigation_state",String,lambda m:states.append(m.data))
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

            robot_position=[0.,0.]

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
                if seed and not empty:
                    back=PoseStamped();back.header=path.header;back.pose.orientation.w=1.
                    back.pose.position.x=.8;back.pose.position.y=-1.;path.poses.append(back)
                (seed_pub if seed else pub).publish(path)
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id = "odom", "base_link"
                odom.header.stamp = rospy.Time.now()
                odom.pose.pose.orientation.w = 1.
                odom.pose.pose.position.x,odom.pose.pose.position.y=robot_position
                tf.transform.translation.x,tf.transform.translation.y=robot_position
                broadcaster.sendTransform(tf)
                odom_pub.publish(odom)

            def moving():
                return any(abs(c.linear.x)+abs(c.angular.z) > 1e-6 for c in commands)

            time.sleep(2.)
            commands.clear();deadline=time.monotonic()+20
            while time.monotonic()<deadline and not moving():feed();time.sleep(.1)
            assert moving(), "Missing sensor freshness still blocks motion"
            # Ensure the layer has explicitly reported its expired update time.
            log.flush();log.seek(0);text=log.read();log.seek(0,2)
            assert "buffer updated in" in text, "Test did not establish a not-current sensor"
            assert "missing_sensor" in [p["name"] for p in rospy.get_param("/placenav_teb_bridge/local_costmap/plugins")]
            print("PASS missing sensor layer remains configured, freshness does not block "+args.mode,flush=True)
            for _ in range(5):feed(empty=True);time.sleep(.1)
            commands.clear()
            for _ in range(5):feed(empty=True);time.sleep(.1)
            assert commands and not moving(), "Empty input no longer stops motion"
            print("PASS empty input still stops",flush=True)
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
