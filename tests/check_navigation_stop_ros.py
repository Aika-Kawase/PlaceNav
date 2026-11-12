#!/usr/bin/env python3
"""Exercise terminal stop on a private master, without any hardware driver."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=11369)
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('127.0.0.1', args.port))
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{args.port}', ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME', None)
    root = Path(__file__).resolve().parents[1]
    children = []
    log = (args.output/'stop.log').open('w')
    clock_running = threading.Event()
    clock_running.set()
    clock_done = threading.Event()

    def start(command):
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        children.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

    def wait(predicate, label, timeout=15):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.02)
        raise AssertionError(label)

    try:
        start(['roscore', '-p', str(args.port)])
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        def ready():
            try:
                master.getPid('/terminal_stop_test')
                return True
            except OSError:
                return False
        wait(ready, 'Master not ready')
        master.setParam('/terminal_stop_test', '/use_sim_time', True)
        import rospy
        import tf2_ros
        import yaml
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Bool, Float32MultiArray
        rospy.init_node('terminal_stop_test', disable_signals=True)
        clock = rospy.Publisher('/clock', Clock, queue_size=1)
        began = time.monotonic()
        def tick():
            while not clock_done.is_set():
                if clock_running.is_set():
                    clock.publish(Clock(clock=rospy.Time.from_sec(100+time.monotonic()-began)))
                time.sleep(.02)
        clock_thread = threading.Thread(target=tick, daemon=True)
        clock_thread.start()
        wait(lambda: rospy.Time.now().to_sec() > 0, 'Clock not received')
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
        tf.transform.rotation.w = 1
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(tf)
        maps = rospy.Publisher('/test/map', OccupancyGrid, queue_size=1, latch=True)
        grid = OccupancyGrid()
        grid.header.frame_id = 'odom'
        grid.info.resolution = .05
        grid.info.width = grid.info.height = 200
        grid.info.origin.position.x = grid.info.origin.position.y = -5
        grid.info.origin.orientation.w = 1
        grid.data = [0]*40000
        maps.publish(grid)
        commands, plans, diagnostics = [], [], []
        rospy.Subscriber('/placenav_teb_bridge/diagnostics', DiagnosticArray,
                         diagnostics.append, queue_size=100)
        rospy.Subscriber('/local_path/cmd_vel', Twist,
                         lambda m: commands.append((time.monotonic(), m)), queue_size=100)
        rospy.Subscriber('/placenav_teb_bridge/active_global_path', RosPath, plans.append)
        path_pub = rospy.Publisher('/test/path', RosPath, queue_size=1)
        odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        def zero(message):
            return all(abs(v) < 1e-9 for v in (
                message.linear.x, message.linear.y, message.linear.z,
                message.angular.x, message.angular.y, message.angular.z))
        def feed():
            path = RosPath()
            path.header.frame_id, path.header.stamp = 'odom', rospy.Time.now()
            for i in range(8):
                point = PoseStamped();point.header = path.header
                point.pose.position.x = .3*i;point.pose.orientation.w = 1
                path.poses.append(point)
            path_pub.publish(path)
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.header.stamp = rospy.Time.now();odom.pose.pose.orientation.w = 1
            odom_pub.publish(odom)
        def configure(mode, retained, goal, stop_topic):
            rospy.set_param('/placenav_teb_bridge', dict(
                reference_mode=mode, retain_safe_path=retained, path_timeout=0.,
                path_topic='/test/path', cmd_vel_topic='/local_path/cmd_vel',
                goal_reached_topic=goal, stop_topic=stop_topic, controller_frequency=5.,
                footprint=[[-.15,-.3],[.85,-.3],[.85,.3],[-.15,.3]],
                local_costmap=dict(global_frame='odom', robot_base_frame='base_link',
                    rolling_window=True, width=10., height=10., resolution=.05,
                    plugins=[dict(name='static', type='costmap_2d::StaticLayer')],
                    static=dict(map_topic='/test/map'), track_unknown_space=False),
                TebLocalPlannerROS=dict(odom_topic='/odom', max_vel_x=.4, max_vel_theta=.4,
                    xy_goal_tolerance=.1, enable_homotopy_class_planning=False)))
        results = []
        cases = [('latest', False, 'goal'), ('state_machine', False, 'goal'),
                 ('latest', True, 'goal'), ('latest', False, 'stop')]
        for index, (mode, retained, event) in enumerate(cases):
            clock_running.set();commands.clear();plans.clear();diagnostics.clear()
            goal, stop_topic = f'/test/{index}/goal', f'/test/{index}/stop'
            goal_pub = rospy.Publisher(goal, Bool, queue_size=1, latch=True)
            stop_pub = rospy.Publisher(stop_topic, Bool, queue_size=1, latch=True)
            configure(mode, retained, goal, stop_topic)
            bridge = start([str(args.bridge)])
            deadline = time.monotonic()+18
            while not any(not zero(m) for _, m in commands) and time.monotonic() < deadline:
                assert bridge.poll() is None, 'Bridge exited'
                feed();time.sleep(.1)
            assert any(not zero(m) for _, m in commands), (
                mode, retained, 'No moving command', str(diagnostics[-1:]))
            clock_running.clear();time.sleep(.08)
            sent = time.monotonic()
            event_pub = goal_pub if event == 'goal' else stop_pub
            event_pub.publish(Bool(data=True))
            wait(lambda: any(t > sent and zero(m) for t, m in commands), 'No terminal zero', 2)
            wait(lambda: plans and not plans[-1].poses, 'Active path not cleared', 2)
            settled = time.monotonic()+.1
            deadline = settled+1.0
            while time.monotonic() < deadline:
                feed()  # A delayed valid path must not reactivate either planner mode.
                event_pub.publish(Bool(data=False))
                time.sleep(.05)
            tail = [(t,m) for t,m in commands if t >= settled]
            assert len(tail) >= 3 and all(zero(m) for _,m in tail), (mode, retained, 'Stop did not persist while paused')
            results.append(dict(mode=mode, retained=retained, event=event,
                                nonzero_before=True, zero_while_paused=len(tail)))
            print('PASS TERMINAL STOP', results[-1], flush=True)
            clock_running.set();stop(bridge);time.sleep(.1)
        # Exercise the actual terminal method, including its own stop subscriber.
        for event in ['goal', 'stop']:
            goal = f'/test/late/{event}/goal';stop_topic = f'/test/late/{event}/stop'
            marker = args.output/f'sender_{event}.ready'
            marker.unlink(missing_ok=True)
            sender_code = (
                "import sys;sys.path[:0]="+repr([str(root/'src'), str(root/'src/placenav')])+";"
                "import rospy;from pathlib import Path;from navigate import PlaceNavNode;"
                "from std_msgs.msg import Bool,Float32MultiArray;rospy.init_node('terminal_sender');"
                "n=PlaceNavNode.__new__(PlaceNavNode);n._terminal_stop_sent=False;"
                "n._goal_reached="+repr(event == 'goal')+";"
                "n.goal_pub=rospy.Publisher("+repr(goal)+",Bool,queue_size=1,latch=True);"
                "n.stop_pub=rospy.Publisher("+repr(stop_topic)+",Bool,queue_size=1,latch=True);"
                "n.waypoint_pub=rospy.Publisher('/test/unused_waypoint',Float32MultiArray,queue_size=1);"
                "rospy.Subscriber("+repr(stop_topic)+",Bool,n.stop_callback);"
                "rospy.on_shutdown(n.publish_terminal_stop);"
                "n.publish_terminal_stop();Path("+repr(str(marker))+").touch();rospy.spin()")
            sender = start(['python3', '-B', '-c', sender_code])
            wait(marker.exists, 'PlaceNav terminal publisher not ready', 30)
            commands.clear();diagnostics.clear();configure('latest', False, goal, stop_topic)
            bridge = start([str(args.bridge)])
            reason = 'NAVIGATION_GOAL_REACHED' if event == 'goal' else 'NAVIGATION_STOP_REQUESTED'
            wait(lambda: any(s.message == reason for d in diagnostics for s in d.status),
                 'Late bridge did not receive latched completion')
            for _ in range(10):feed();time.sleep(.05)
            assert sender.poll() is None, 'Own stop callback closed the terminal publisher'
            assert commands and all(zero(m) for _,m in commands)
            stop(bridge);stop(sender)
            results.append(dict(late_bridge=True, event=event, all_zero=True))
            print('PASS LATE BRIDGE RECEIVES PLACENAV TERMINAL EVENT', event, flush=True)
        for index, event in enumerate(['goal', 'stop']):
            commands.clear();clock_running.set()
            goal, stop_topic = f'/test/pd{index}/goal', f'/test/pd{index}/stop'
            config = dict(whill=dict(max_v=.4, max_w=.4, frame_rate=5,
                waypoint_topic='/test/waypoint', reached_goal_topic=goal, stop_topic=stop_topic,
                vel_navi_topic='/local_path/cmd_vel'))
            path = args.output/f'pd{index}.yaml';path.write_text(yaml.safe_dump(config))
            goal_pub = rospy.Publisher(goal, Bool, queue_size=1)
            stop_pub = rospy.Publisher(stop_topic, Bool, queue_size=1)
            waypoints = rospy.Publisher('/test/waypoint', Float32MultiArray, queue_size=1)
            pd = start(['python3', '-B', str(root/'src/placenav/pd_controller.py'),
                        '--robot', 'whill', '--robot-config-path', str(path)])
            end = time.monotonic()+8
            while not any(not zero(m) for _,m in commands) and time.monotonic()<end:
                waypoints.publish(Float32MultiArray(data=[.3, 0]));time.sleep(.05)
            assert any(not zero(m) for _,m in commands), 'PD never moved'
            clock_running.clear();time.sleep(.08)
            event_pub = goal_pub if event=='goal' else stop_pub
            sent=time.monotonic();event_pub.publish(Bool(data=True))
            wait(lambda: any(t>sent and zero(m) for t,m in commands), 'PD did not stop', 2)
            settled=time.monotonic()+.1
            for _ in range(10):
                event_pub.publish(Bool(data=False))
                waypoints.publish(Float32MultiArray(data=[.3,.1]));time.sleep(.05)
            clock_running.set();time.sleep(.4)
            tail=[m for t,m in commands if t>=settled]
            assert tail and all(zero(m) for m in tail), 'Late waypoint resumed PD command'
            stop(pd)
            results.append(dict(pd=True,event=event,all_zero=True))
            print('PASS PD TERMINAL STOP', event, flush=True)
        (args.output/'result.json').write_text(json.dumps(results,indent=2)+'\n')
    except Exception:
        log.flush()
        print((args.output/'stop.log').read_text()[-7000:],flush=True)
        raise
    finally:
        clock_running.set()
        for process in reversed(children):stop(process)
        clock_done.set()
        log.close()


if __name__ == '__main__':
    main()
