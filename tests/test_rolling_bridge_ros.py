#!/usr/bin/env python3
"""Isolated ROS integration test. Never connects to the robot's ROS master.

Run in the ROS container with --binary pointing to the newly built bridge.
Only /rolling_test/cmd_vel is used. A synthetic static costmap is supplied.
"""
import argparse
import math
import os
import socket
import subprocess
import tempfile
import threading
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True)
    args = parser.parse_args()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    log = tempfile.TemporaryFile(mode='w+')
    master = subprocess.Popen(['roscore', '-p', str(port)], stdout=log, stderr=log)
    bridge = None
    quit_event = threading.Event()
    try:
        import rosgraph
        deadline = time.monotonic()+15
        while time.monotonic()<deadline:
            try:
                rosgraph.Master('/test').getPid()
                break
            except Exception:
                time.sleep(.1)
        else:
            raise RuntimeError('isolated master did not start')
        import rospy
        import tf2_ros
        import yaml
        from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Path, Odometry
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Header, Bool, String
        from std_srvs.srv import Trigger
        from rosgraph_msgs.msg import Clock
        rospy.init_node('rolling_test', anonymous=True, disable_signals=True)
        config_path = os.path.join(os.path.dirname(__file__), '../src/placenav_teb_bridge/config/bridge.yaml')
        with open(config_path) as stream:
            config = yaml.safe_load(stream)
        config['cmd_vel_topic'] = '/rolling_test/cmd_vel'
        config['local_costmap'].update({
            'rolling_window': False,
            'plugins': [{'name': 'static', 'type': 'costmap_2d::StaticLayer'}],
            'static': {'map_topic': '/rolling_test/map', 'subscribe_to_updates': True},
        })
        # No real sensors, STVL, motor controller or existing ROS master are used.
        rospy.set_param('/placenav_teb_bridge', config)
        map_pub = rospy.Publisher('/rolling_test/map', OccupancyGrid, queue_size=1, latch=True)
        path_pub = rospy.Publisher('/placenav/gnm_path', Path, queue_size=1)
        odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        cloud_pub = rospy.Publisher('/c2/depth/points', PointCloud2, queue_size=1)
        goal_pub = rospy.Publisher('/toponav/reached_goal', Bool, queue_size=1, latch=True)
        clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
        tf_pub = tf2_ros.TransformBroadcaster()
        state = {'robot': (0., 0., 0.), 'cloud': True, 'odom': True, 'tf': True,
                 'path': True, 'obstacle': None, 'status': '', 'mode': '', 'active': None,
                 'path_stamp_offset': 0., 'commands': [], 'path_length': 2., 'clock': False}
        def on_status(m): state['status'] = m.data
        def on_mode(m): state['mode'] = m.data
        def on_path(m): state['active'] = m
        def on_cmd(m): state['commands'].append((time.monotonic(), m.linear.x, m.angular.z))
        rospy.Subscriber('/placenav_teb_bridge/control_status', String, on_status)
        rospy.Subscriber('/placenav_teb_bridge/navigation_state', String, on_mode)
        rospy.Subscriber('/placenav_teb_bridge/active_global_path', Path, on_path)
        rospy.Subscriber('/rolling_test/cmd_vel', Twist, on_cmd)
        def publish_map():
            m = OccupancyGrid()
            m.header.frame_id = 'odom'; m.header.stamp = rospy.Time.now()
            m.info.resolution = .05; m.info.width = m.info.height = 240
            m.info.origin.position.x = m.info.origin.position.y = -6
            m.info.origin.orientation.w = 1
            m.data = [0]*(240*240)
            if state['obstacle']=='wall':
                ix=int((1.2+6)/.05)
                for iy in range(240):m.data[iy*240+ix]=100
            elif state['obstacle'] is not None:
                ox, oy = state['obstacle']
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        ix = int((ox+6)/.05)+dx; iy = int((oy+6)/.05)+dy
                        if 0<=ix<240 and 0<=iy<240: m.data[iy*240+ix] = 100
            map_pub.publish(m)
        def feed():
            while not quit_event.is_set():
                stamp = rospy.Time.now(); x, y, yaw = state['robot']
                if state['clock']:clock_pub.publish(Clock(clock=stamp))
                tf = TransformStamped(); tf.header.frame_id='odom';tf.child_frame_id='base_link';tf.header.stamp=stamp
                tf.transform.translation.x=x;tf.transform.translation.y=y
                tf.transform.rotation.z=math.sin(yaw/2);tf.transform.rotation.w=math.cos(yaw/2)
                if state['tf']: tf_pub.sendTransform(tf)
                odom = Odometry();odom.header=tf.header;odom.child_frame_id='base_link'
                odom.pose.pose.position.x=x;odom.pose.pose.position.y=y
                odom.pose.pose.orientation=tf.transform.rotation
                if state['odom']: odom_pub.publish(odom)
                if state['cloud']:
                    cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='odom'), [(3.,3.,1.)]))
                path = Path();path.header.frame_id='odom'
                path.header.stamp=stamp+rospy.Duration(state['path_stamp_offset'])
                for i in range(21):
                    p=PoseStamped();p.header=path.header
                    step=i*state['path_length']/20
                    p.pose.position.x=x+step*math.cos(yaw);p.pose.position.y=y+step*math.sin(yaw)
                    p.pose.orientation=tf.transform.rotation;path.poses.append(p)
                if state['path']: path_pub.publish(path)
                quit_event.wait(.05)
        thread = threading.Thread(target=feed, daemon=True); thread.start()
        publish_map()
        bridge = subprocess.Popen([args.binary, '__name:=placenav_teb_bridge'], stdout=log, stderr=log)
        def wait_for(condition, label, timeout=12):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                if bridge.poll() is not None: raise AssertionError('bridge exited')
                if condition():
                    print('PASS:', label, flush=True);return
                time.sleep(.05)
            raise AssertionError('%s; status=%s mode=%s' % (label,state['status'],state['mode']))
        wait_for(lambda: state['status']=='RUNNING', 'free route running')
        state['cloud']=False
        wait_for(lambda: state['status']=='INPUT_STALE_CLOUD', 'stale cloud stops')
        state['cloud']=True
        wait_for(lambda: state['status']=='RUNNING', 'fresh cloud resumes')
        state['odom']=False
        wait_for(lambda: state['status']=='INPUT_STALE_ODOM', 'stale odometry stops')
        state['odom']=True
        wait_for(lambda: state['status']=='RUNNING', 'fresh odometry resumes')
        state['tf']=False
        wait_for(lambda: state['status'] in ('TF_UNAVAILABLE','TF_OR_COSTMAP_UNAVAILABLE'), 'missing TF stops')
        state['tf']=True
        wait_for(lambda: state['status']=='RUNNING', 'TF resumes')
        state['path_stamp_offset']=-3
        wait_for(lambda: state['status']=='INPUT_STALE_PATH', 'old input timestamp stops')
        state['path_stamp_offset']=0
        wait_for(lambda: state['status']=='RUNNING', 'fresh path resumes')
        goal_pub.publish(True)
        wait_for(lambda: state['status']=='FINAL_GOAL_REACHED', 'final goal distinct')
        since=time.monotonic();goal_pub.publish(False);time.sleep(.7)
        assert all(abs(x)+abs(w)<1e-9 for t,x,w in state['commands'] if t>since+.1), 'latched final goal must stay stopped'
        rospy.wait_for_service('/placenav_teb_bridge/reset_navigation', timeout=5)
        rospy.ServiceProxy('/placenav_teb_bridge/reset_navigation', Trigger)()
        wait_for(lambda: state['status']=='RUNNING', 'explicit reset and new path resume')
        state['obstacle']=(1.8,0);publish_map()
        wait_for(lambda: state['mode']=='AVOIDING' and state['status']=='RUNNING' and
                 state['active'] and any(abs(p.pose.position.y)>.3 for p in state['active'].poses),
                 'obstacle detour accepted',20)
        active=state['active']
        assert active and any(abs(p.pose.position.y)>.3 for p in active.poses), 'detour must go around obstacle'
        # Advance along the generated path near its end, rather than extending a fixed seed.
        target=active.poses[max(0,len(active.poses)-9)].pose
        old_end=active.poses[-1].pose.position
        yaw=2*math.atan2(target.orientation.z,target.orientation.w)
        state['robot']=(target.position.x,target.position.y,yaw)
        state['obstacle']=(target.position.x+1.8*math.cos(yaw),target.position.y+1.8*math.sin(yaw));publish_map()
        wait_for(lambda: state['active'] and math.hypot(state['active'].poses[-1].pose.position.x-old_end.x,
                    state['active'].poses[-1].pose.position.y-old_end.y)>.5 and state['status']=='RUNNING',
                 'near endpoint obtains a new goal',20)
        state['obstacle']=None;publish_map()
        wait_for(lambda: state['mode']=='RECOVERING', 'safe connection recovery')
        wait_for(lambda: state['mode']=='NORMAL', 'stable recovery normal')
        state['robot']=(0.,0.,0.);state['obstacle']='wall';publish_map()
        wait_for(lambda: state['status']=='PLANNING_FAILED', 'unreachable route planning failure')
        since=time.monotonic();time.sleep(.4)
        assert all(abs(x)+abs(w)<1e-9 for t,x,w in state['commands'] if t>since+.1)
        state['obstacle']=None;publish_map()
        wait_for(lambda: state['mode']=='NORMAL' and state['status']=='RUNNING', 'clear route resumes')
        end=state['active'].poses[-1].pose
        state['robot']=(end.position.x,end.position.y,2*math.atan2(end.orientation.z,end.orientation.w))
        state['path_length']=.1
        wait_for(lambda: state['status']=='ENDPOINT_WAIT', 'endpoint wait distinct from planning failure')
        state['path_length']=2.;state['robot']=(0.,0.,0.)
        wait_for(lambda: state['status']=='RUNNING', 'new forward path leaves endpoint wait')
        # A ROS timer stops when /clock pauses. The wall watchdog must still stop.
        bridge.terminate();bridge.wait(timeout=8);bridge=None
        rospy.set_param('/use_sim_time', True)
        state['status']='';state['clock']=True
        bridge=subprocess.Popen([args.binary,'__name:=placenav_teb_bridge'],stdout=log,stderr=log)
        wait_for(lambda: state['status']=='RUNNING', 'sim-time control running')
        state['clock']=False
        wait_for(lambda: state['status']=='CLOCK_OR_CONTROL_STALLED', 'paused clock wall watchdog stop')
        state['clock']=True
        wait_for(lambda: state['status']=='RUNNING', 'clock resumes with fresh inputs')
        print('All isolated ROS checks passed.', flush=True)
    except Exception:
        log.seek(0)
        print(log.read()[-14000:])
        raise
    finally:
        quit_event.set()
        for proc in (bridge,master):
            if proc and proc.poll() is None:
                proc.terminate()
                try: proc.wait(timeout=8)
                except subprocess.TimeoutExpired: proc.kill();proc.wait()
        log.close()


if __name__=='__main__': main()
