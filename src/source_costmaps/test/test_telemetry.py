#!/usr/bin/env python3
"""Exercise trace publication and the real path adapter with ROS messages/TF."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Path as NavPath
from placenav_viz_msgs.msg import Viz
from std_msgs.msg import String
import tf2_ros

sys.path.insert(0,'/work/external/PlaceNav/src/placenav')
from navigation_trace import NavigationTrace
from gnm_path_adapter import GnmPathAdapter


class TelemetryTest(unittest.TestCase):
    def wait(self,predicate):
        end=time.monotonic()+8
        while time.monotonic()<end:
            if predicate(): return
            time.sleep(.02)
        self.fail('ROS telemetry timeout')

    def test_metadata_input_stamp_and_transformed_path_are_published(self):
        messages={}
        topics={'metadata':'/rwrc26/navigation_metadata','trace':'/rwrc26/navigation_trace',
                'adapter':'/rwrc26/path_adapter_trace','path':'/validation/gnm_path'}
        subs=[rospy.Subscriber(topic,NavPath if key=='path' else String,
            lambda msg,key=key:messages.__setitem__(key,msg),queue_size=10) for key,topic in topics.items()]
        try:
            with tempfile.TemporaryDirectory(prefix='trace-fixture-',dir='/work/results/navigation_validation') as directory:
                root=Path(directory); (root/'weights').write_bytes(b'synthetic telemetry fixture, not model inference')
                (root/'models.yaml').write_text('nomad:\n  checkpoint_path: weights\nsalad:\n  model:\n    checkpoint_path: weights\n')
                (root/'robot.yaml').write_text('fixture: true\n')
                args=argparse.Namespace(wp_model='nomad',pr_model='salad',model_config_path=root/'models.yaml',
                    robot_config_path=root/'robot.yaml',model_weight_dir=root)
                trace=NavigationTrace(True,args,'fixture_no_model_loaded')
                self.wait(lambda:trace.publisher.get_num_connections()>0)
                trace.emit('smoke',synthetic_fixture=True,query_ns=rospy.Time.now().to_nsec())
                self.wait(lambda:'metadata' in messages and 'trace' in messages)
                metadata=json.loads(messages['metadata'].data)
                self.assertEqual(len(metadata['weights']),2)
                self.assertEqual(metadata['args']['wp_model'],'nomad')
                self.assertEqual(metadata['args']['pr_model'],'salad')
                self.assertEqual(metadata['weights'][0]['sha256'],metadata['weights'][1]['sha256'])
                rospy.set_param('~input_topic','/validation/viz'); rospy.set_param('~output_topic',topics['path'])
                rospy.set_param('~trace_topic',topics['adapter']); rospy.set_param('~source_frame','base_link')
                rospy.set_param('~target_frame','odom'); rospy.set_param('~expected_waypoints',2)
                adapter=GnmPathAdapter(); broadcaster=tf2_ros.StaticTransformBroadcaster()
                tf=TransformStamped(); tf.header.frame_id='odom'; tf.child_frame_id='base_link'
                tf.transform.translation.x=2.; tf.transform.rotation.w=1; broadcaster.sendTransform(tf)
                pub=rospy.Publisher('/validation/viz',Viz,queue_size=1)
                self.wait(lambda:pub.get_num_connections()>0)
                time.sleep(.2)
                viz=Viz(); viz.query_timestamp=rospy.Time.now(); viz.waypoints=[Point(x=1.),Point(x=2.)]
                pub.publish(viz); self.wait(lambda:'path' in messages and 'adapter' in messages)
                self.assertEqual(messages['path'].header.stamp,viz.query_timestamp)
                self.assertEqual(len(messages['path'].poses),3)
                self.assertAlmostEqual(messages['path'].poses[-1].pose.position.x,4.)
                event=json.loads(messages['adapter'].data)
                self.assertEqual(event['query_ns'],viz.query_timestamp.to_nsec())
                self.assertGreaterEqual(event['published_ros_ns'],event['received_ros_ns'])
                result=dict(metadata_hash_count=len(metadata['source_sha256']),adapter_event=event,
                            path_stamp_preserved=True,transform_and_origin_verified=True,
                            fixture='Synthetic messages only; no model inference or robot commands')
                Path('/work/results/navigation_validation/telemetry_smoke.json').write_text(json.dumps(result,indent=2)+'\n')
                adapter.viz_subscriber.unregister()
        finally:
            for sub in subs:sub.unregister()


if __name__=='__main__':
    rospy.init_node('test_navigation_telemetry')
    rostest.rosrun('source_costmaps','navigation_telemetry',TelemetryTest)
