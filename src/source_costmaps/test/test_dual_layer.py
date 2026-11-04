#!/usr/bin/env python3
"""Stationary synthetic sensor test of the actual dual-layer TEB costmap.

Commands stay on /fusion_validation/cmd_vel. No robot/driver or recorded command
input is connected. This is a map/planner response check, not driving success.
"""
import json
import hashlib
import math
from pathlib import Path
import threading
import time
import unittest

import numpy as np
import rospy
import rostest
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rosgraph_msgs.msg import Clock
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_matrix


class DualLayerRuntime(unittest.TestCase):
    def setUp(self):
        self.out = Path(rospy.get_param('~output_dir'))
        self.out.mkdir(parents=True, exist_ok=False)
        self.lock = threading.RLock(); self.now = 1000.; self.running = True
        self.snapshots = {}
        self.grid = None; self.commands = []; self.traces = []; self.states = []; self.stages = []
        self.clock = rospy.Publisher('/clock', Clock, queue_size=1)
        self.tf = rospy.Publisher('/tf', TFMessage, queue_size=5)
        self.static = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.path = rospy.Publisher('/placenav/gnm_path', NavPath, queue_size=1)
        self.inputs = {s: rospy.Publisher(t, PointCloud2, queue_size=1) for s,t in
            [('d435','/realsense/depth/color/points'), ('unik3d','/c2/depth/points')]}
        self.subs = [rospy.Subscriber('/placenav_teb_bridge/local_costmap/costmap',OccupancyGrid,self.on_grid),
            rospy.Subscriber('/fusion_validation/cmd_vel',Twist,self.on_command),
            rospy.Subscriber('/rwrc26/teb_trace',String,self.on_trace),
            rospy.Subscriber('/placenav_teb_bridge/navigation_state',String,lambda m:self.states.append(m.data))]
        self.origin = np.array([.02652, 0, 1.4686620009085125])
        self.rotation = quaternion_matrix([-.4813627040827183,.4865498075348136,-.5130442130488477,.5180201418507755])[:3,:3]
        self.static.publish(TFMessage([self.transform('base_link','realsense_color_optical_frame',[0,0,1]),
            self.transform('base_link','c2_camera_optical_frame',[.02652,0,1.0175])]))
        def pump():
            while self.running and not rospy.is_shutdown():
                with self.lock:
                    self.now += .02; now = self.now
                self.clock.publish(Clock(rospy.Time.from_sec(now)))
                self.tf.publish(TFMessage([self.transform('odom','base_link',[0,0,0],now)]))
                od = Odometry(); od.header = Header(stamp=rospy.Time.from_sec(now),frame_id='odom')
                od.child_frame_id='base_link'; od.pose.pose.orientation.w=1; self.odom.publish(od)
                time.sleep(.02)
        self.thread = threading.Thread(target=pump, daemon=True); self.thread.start()
        self.wait(lambda: self.path.get_num_connections() and all(p.get_num_connections() for p in self.inputs.values()),20)
        self.wait(lambda: self.grid is not None,10)

    @staticmethod
    def transform(parent, child, position, stamp=0):
        t=TransformStamped(); t.header.frame_id=parent; t.child_frame_id=child
        t.header.stamp=rospy.Time.from_sec(stamp)
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=position
        t.transform.rotation.w=1; return t

    def wait(self, condition, timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if condition(): return
            time.sleep(.025)
        self.fail('Timeout waiting for runtime state')

    def on_grid(self,msg):
        with self.lock: self.grid=msg

    def on_command(self,msg):
        with self.lock: self.commands.append(dict(t=self.now,v=msg.linear.x,w=msg.angular.z))

    def on_trace(self,msg):
        with self.lock: self.traces.append(json.loads(msg.data))

    def occupied(self,x,y):
        with self.lock: m=self.grid
        ix=int(math.floor((x-m.info.origin.position.x)/m.info.resolution))
        iy=int(math.floor((y-m.info.origin.position.y)/m.info.resolution))
        return any(0<=ix+dx<m.info.width and 0<=iy+dy<m.info.height and
            m.data[(iy+dy)*m.info.width+ix+dx]==100 for dx in (-1,0,1) for dy in (-1,0,1))

    def stage(self,name,d435,unik3d,duration=2.0):
        start=self.now; until=time.monotonic()+duration
        while time.monotonic()<until:
            with self.lock: stamp=self.now-.04
            for source,points in [('d435',d435),('unik3d',unik3d)]:
                arr=np.asarray(points,dtype=float).reshape(-1,3)
                if source=='d435':
                    arr=arr-np.array([0,0,1]); frame='realsense_color_optical_frame'
                else:
                    arr=(arr-self.origin)@self.rotation; arr=np.repeat(arr,3,axis=0); frame='c2_camera_optical_frame'
                cloud=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.from_sec(stamp),frame_id=frame),arr)
                self.inputs[source].publish(cloud)
            path=NavPath(); path.header=Header(stamp=rospy.Time.from_sec(stamp),frame_id='odom')
            for x in np.linspace(0,3,9):
                pose=PoseStamped(); pose.header=path.header; pose.pose.position.x=x; pose.pose.orientation.w=1
                path.poses.append(pose)
            self.path.publish(path); time.sleep(.1)
        with self.lock:
            commands=[r for r in self.commands if r['t']>=start+.5]
            grid=self.grid
            result=dict(stage=name,start_s=start,end_s=self.now,commands=len(commands),
                nonzero_commands=sum(abs(r['v'])>1e-5 or abs(r['w'])>1e-5 for r in commands),
                max_abs_v=max([abs(r['v']) for r in commands] or [0]),
                unknown_cells=sum(v==-1 for v in grid.data),lethal_cells=sum(v==100 for v in grid.data))
            self.stages.append(result)
            self.snapshots[name]=np.asarray(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width)

    def test_obstacle_motion_expiry_unknown_and_retention(self):
        self.stage('empty_unknown',[],[])
        self.assertGreater(self.stages[-1]['unknown_cells'],0)
        self.stage('d435_low_obstacle',[[1.225,0,.225],[1.225,-.775,.025]],[])
        self.assertTrue(self.occupied(1.225,0))
        self.assertFalse(self.occupied(1.225,-.775))
        self.stage('obstacle_moved',[[1.225,.825,.225]],[])
        self.assertTrue(self.occupied(1.225,.825))
        self.assertFalse(self.occupied(1.225,0))
        self.stage('both_sources',[[1.225,.825,.225]],[[2.225,-1.225,.725]])
        self.assertTrue(self.occupied(1.225,.825)); self.assertTrue(self.occupied(2.225,-1.225))
        self.stage('d435_only',[[1.225,.825,.225]],[])
        self.assertTrue(self.occupied(1.225,.825)); self.assertFalse(self.occupied(2.225,-1.225))
        self.stage('all_expired',[],[])
        self.assertFalse(self.occupied(1.225,.825)); self.assertFalse(self.occupied(2.225,-1.225))
        self.assertGreater(self.stages[-1]['unknown_cells'],0)
        self.assertTrue(any(r['reason']=='equivalent_retained' for r in self.traces))
        self.assertTrue(any(r['event']=='control' and r['active_input_ns']>0 for r in self.traces))

    def tearDown(self):
        self.running=False; self.thread.join(2)
        for sub in self.subs: sub.unregister()
        (self.out/'summary.json').write_text(json.dumps(dict(stages=self.stages,
            navigation_states=self.states,trace_events=len(self.traces),
            limits='Stationary synthetic clouds and odometry; no physical robot or closed-loop motion. Nonzero command in unknown space is an observed configured behavior, not a safety acceptance.'),indent=2)+'\n')
        (self.out/'teb_trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.traces))
        (self.out/'commands.json').write_text(json.dumps(self.commands,indent=2)+'\n')
        np.savez_compressed(self.out/'stage_grids.npz',**self.snapshots)
        (self.out/'effective_parameters.json').write_text(json.dumps(rospy.get_param('/placenav_teb_bridge'),indent=2)+'\n')
        (self.out/'test_source_sha256.txt').write_text(hashlib.sha256(Path(__file__).read_bytes()).hexdigest()+'\n')


if __name__=='__main__':
    rospy.init_node('test_dual_layer_runtime')
    rostest.rosrun('source_costmaps','dual_layer_runtime',DualLayerRuntime)
