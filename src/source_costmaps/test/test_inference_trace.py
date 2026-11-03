#!/usr/bin/env python3
"""Recorded images -> actual SALAD/NoMaD -> adapter -> isolated two-layer TEB.

17-node writable map fixture, synthetic stationary TF/odom, validation-only
commands. Verifies telemetry wiring/provenance, not navigation improvement.
"""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import unittest

import cv2
import numpy as np
import rosbag
import rospy
import rostest
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from placenav_viz_msgs.msg import Viz
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import String
import tf2_ros
import torch


class RealModelTrace(unittest.TestCase):
    def test_image_provenance_and_all_runtime_stages(self):
        root=Path('/work/results/navigation_validation'); fixture=root/'runtime_inputs'
        out=root/'runtime_trace'; out.mkdir(exist_ok=False)
        data=json.loads((fixture/'frames.json').read_text()); frames=data['frames']
        first=frames[0]['stamp_ns']; lock=threading.RLock(); running=True; playback=None
        predictions=[]; events=[]; metadata=[]; expected={}; paths=[]; owned=[]
        writer=rosbag.Bag(str(out/'trace.bag'),'w')
        def receive(topic,msg):
            with lock:
                writer.write(topic,msg,rospy.Time.now())
                if topic=='/rwrc26/navigation_metadata': metadata.append(json.loads(msg.data))
                elif topic.startswith('/rwrc26/'):
                    row=json.loads(msg.data); events.append(row)
                    if row['event']=='prediction': predictions.append(row)
                elif topic=='/placenav/gnm_path': paths.append(msg)
        topics={'/rwrc26/navigation_metadata':String,'/rwrc26/navigation_trace':String,
            '/rwrc26/path_adapter_trace':String,'/rwrc26/teb_trace':String,
            '/toponav/viz_info':Viz,'/placenav/gnm_path':NavPath,'/fusion_validation/cmd_vel':Twist}
        subs=[rospy.Subscriber(t,typ,lambda msg,t=t:receive(t,msg),queue_size=100) for t,typ in topics.items()]
        clock=rospy.Publisher('/clock',Clock,queue_size=1); odom=rospy.Publisher('/odom',Odometry,queue_size=1)
        images=rospy.Publisher('/usb_cam/image_raw',Image,queue_size=1)
        tf=tf2_ros.StaticTransformBroadcaster(); transform=TransformStamped()
        transform.header.frame_id='odom'; transform.child_frame_id='base_link'; transform.transform.rotation.w=1
        tf.sendTransform(transform)
        def pump():
            while running and not rospy.is_shutdown():
                ns=first-1_000_000_000 if playback is None else first+int((time.monotonic()-playback)*1e9)
                stamp=rospy.Time(ns//1_000_000_000,ns%1_000_000_000)
                clock.publish(Clock(stamp)); message=Odometry(); message.header.stamp=stamp
                message.header.frame_id='odom'; message.child_frame_id='base_link'; message.pose.pose.orientation.w=1
                odom.publish(message); time.sleep(.02)
        thread=threading.Thread(target=pump,daemon=True); thread.start()
        logs=[]
        command=['python3','/work/external/PlaceNav/src/placenav/navigate.py','--robot','whill',
            '--robot-config-path','/work/external/PlaceNav/src/config/robots.yaml',
            '--model-config-path',str(fixture/'models.yaml'),'--model-weight-dir','/work',
            '--topomap-base-dir',str(fixture/'maps'),'--topomap-dir','integration_map',
            '--wp-model','nomad','--pr-model','salad','--filter-mode','bayesian',
            '--subgoal-mode','place_recognition','--device','cpu','--trace-navigation']
        try:
            for name,args in [('navigation',command),('adapter',['python3','/work/external/PlaceNav/src/placenav/gnm_path_adapter.py',
                '--expected-waypoints','8','--trace-topic','/rwrc26/path_adapter_trace'])]:
                stream=(out/f'{name}.log').open('w'); logs.append(stream)
                owned.append(subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
            ready=fixture/'maps/integration_map/global-feats-salad.h5.ready'
            deadline=time.monotonic()+75
            while not (ready.exists() and images.get_num_connections()>0 and metadata):
                for process in owned:
                    if process.poll() is not None:
                        self.fail((out/'navigation.log').read_text()+'\n'+(out/'adapter.log').read_text())
                self.assertLess(time.monotonic(),deadline,'Runtime model startup timeout')
                time.sleep(.05)
            playback=time.monotonic()
            for row in frames:
                due=(row['stamp_ns']-first)/1e9
                while time.monotonic()-playback<due: time.sleep(.005)
                path=Path(data['image_root'])/row['file']
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),row['sha256'])
                rgb=cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB)
                msg=Image(); ns=row['stamp_ns']; msg.header.stamp=rospy.Time(ns//1_000_000_000,ns%1_000_000_000)
                msg.height,msg.width=rgb.shape[:2]; msg.encoding='rgb8'; msg.step=msg.width*3; msg.data=rgb.tobytes()
                expected[ns]=hashlib.sha256(msg.data).hexdigest(); images.publish(msg)
            time.sleep(1.0)
            self.assertGreaterEqual(len(predictions),5)
            self.assertTrue(paths)
            self.assertTrue(any(e['event']=='path_adapter' for e in events))
            self.assertTrue(any(e['event']=='path_received' for e in events))
            self.assertTrue(any(e['event']=='control' and e['active_input_ns']>0 for e in events))
            for row in predictions:
                self.assertEqual(len(row['history_stamp_ns']),4)
                self.assertEqual(row['history_stamp_ns'][-1],row['query_ns'])
                for ns,h in zip(row['history_stamp_ns'],row['history_raw_sha256']): self.assertEqual(h,expected[ns])
                self.assertGreaterEqual(row['waypoint_finished_ros_ns'],row['waypoint_started_ros_ns'])
            self.assertEqual(metadata[-1]['args']['wp_model'],'nomad')
            self.assertEqual(metadata[-1]['versions']['torch'],torch.__version__)
            (out/'summary.json').write_text(json.dumps(dict(frames_sent=len(frames),predictions=len(predictions),
                path_messages=len(paths),event_counts={name:sum(e['event']==name for e in events) for name in ('prediction','path_adapter','path_received','control')},
                raw_history_hashes_verified=True,model_metadata=metadata[-1],
                limits='17-node local map; actual recorded image inference with CPU torch 2.4.1; synthetic stationary TF/odom and marking-only map. No robot connected; not a historical or closed-loop driving reproduction.'),indent=2)+'\n')
        finally:
            for process in owned:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try: process.wait(timeout=10)
                    except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGTERM); process.wait(timeout=3)
            running=False; thread.join(2)
            for sub in subs: sub.unregister()
            with lock: writer.close()
            for stream in logs: stream.close()


if __name__=='__main__':
    rospy.init_node('test_real_model_trace')
    rostest.rosrun('source_costmaps','real_model_trace',RealModelTrace)
