#!/usr/bin/env python3
"""Run the actual map generator with timestamped ROS images and odometry."""
import json
from pathlib import Path
import pickle
import subprocess
import tempfile
import time
import unittest

import numpy as np
import rospy
import rostest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


class TopomapSyncTest(unittest.TestCase):
    def test_saved_images_use_bracketed_image_time_pose_and_do_not_overwrite(self):
        with tempfile.TemporaryDirectory(dir='/work/results/navigation_validation',prefix='map-sync-fixture-') as directory:
            root=Path(directory); config=root/'robot.yaml'
            config.write_text('fixture:\n  camera_topic: /validation/map_image\n  odom_topic: /validation/map_odom\n')
            command=['python3','/work/external/PlaceNav/src/placenav/create_topomap.py','--robot','fixture',
                '--robot_config_path',str(config),'--topomap_directory',str(root),'--route_name','new_map','--dt','0.5']
            image_pub=rospy.Publisher('/validation/map_image',Image,queue_size=1)
            odom_pub=rospy.Publisher('/validation/map_odom',Odometry,queue_size=10)
            with (root/'generator.log').open('w') as stream:
                process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT)
                try:
                    deadline=time.monotonic()+10
                    while not (image_pub.get_num_connections() and odom_pub.get_num_connections()):
                        if process.poll() is not None: self.fail((root/'generator.log').read_text())
                        self.assertLess(time.monotonic(),deadline); time.sleep(.01)
                    expected={}
                    for i in range(3):
                        # Pose moves four meters over 40 ms: image at the midpoint
                        # must save x=2+10i, rather than the latest pose x=4+10i.
                        stamp=rospy.Time.now().to_nsec()-100_000_000
                        for dt,x in ((0,10*i),(40_000_000,10*i+4)):
                            od=Odometry(); od.header.stamp=rospy.Time.from_sec(0)
                            od.header.stamp.secs=(stamp+dt)//1_000_000_000
                            od.header.stamp.nsecs=(stamp+dt)%1_000_000_000
                            od.pose.pose.position.x=x; od.pose.pose.orientation.w=1
                            odom_pub.publish(od)
                        time.sleep(.04)
                        im=Image(); im.header.stamp.secs=(stamp+20_000_000)//1_000_000_000
                        im.header.stamp.nsecs=(stamp+20_000_000)%1_000_000_000
                        im.height=8; im.width=8; im.encoding='rgb8'; im.step=24
                        im.data=bytes([20+50*i,40,60])*64; image_pub.publish(im)
                        expected[im.header.stamp.to_nsec()]=2.+10*i
                        time.sleep(.6)
                    process.wait(timeout=5)
                    self.assertEqual(process.returncode,0,(root/'generator.log').read_text())
                finally:
                    if process.poll() is None: process.terminate(); process.wait(timeout=3)
            path=root/'new_map/traj_data.pkl'
            with path.open('rb') as stream: trajectory=pickle.load(stream)
            self.assertEqual(len(trajectory['position']),3)
            for stamp,xy in zip(trajectory['image_stamp_ns'],trajectory['position']):
                self.assertAlmostEqual(xy[0],expected[stamp],places=9)
            np.testing.assert_allclose(np.asarray(trajectory['relative_position'],dtype=float),[[0,0],[10,0],[10,0]])
            original=path.read_bytes()
            rerun=subprocess.run(command,capture_output=True,text=True,timeout=5)
            self.assertNotEqual(rerun.returncode,0)
            self.assertEqual(path.read_bytes(),original)
            result=dict(saved_images=3,interpolated_image_time_positions_verified=True,
                        relative_saved_node_deltas_verified=True,existing_map_preserved=True,
                        source='actual create_topomap.py in a separate ROS process',
                        limits='Synthetic images/odometry, not physical pose truth')
            Path('/work/results/navigation_validation/topomap_runtime.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':
    rospy.init_node('test_topomap_image_time')
    rostest.rosrun('source_costmaps','topomap_image_time',TopomapSyncTest)
