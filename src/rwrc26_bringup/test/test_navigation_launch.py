#!/usr/bin/env python3
"""Expand the real launch with ROS, without starting inference or controllers."""
import json
import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from roslaunch.config import load_config_default


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT/'launch/navigation.launch'


class NavigationLaunchTest(unittest.TestCase):
    def load(self, *args):
        config = load_config_default([(str(LAUNCH), list(args))], 11311, verbose=False)
        return config, {name: param.value for name, param in config.params.items()}

    def test_vpr_transition_window_defaults_and_override(self):
        _, params = self.load()
        prefix = '/placenav_runner/transition_model_window_'
        self.assertEqual(params[prefix+'lower'], 0)
        self.assertEqual(params[prefix+'upper'], 2)
        _, params = self.load('transition_model_window_lower:=-1', 'transition_model_window_upper:=3')
        self.assertEqual(params[prefix+'lower'], -1)
        self.assertEqual(params[prefix+'upper'], 3)

    def test_modes_and_reference_controls_reach_actual_teb(self):
        snapshots = {}
        for fusion, profile in (('layers', 'current'), ('layers', 'matched'), ('points', 'matched')):
            for reference in ('state_machine', 'latest'):
                with self.subTest(fusion=fusion, profile=profile, reference=reference):
                    config, params = self.load('fusion_mode:='+fusion, 'comparison_profile:='+profile,
                                               'reference_mode:='+reference, 'path_timeout:=1.25',
                                               'shrink_horizon_backup:=false')
                    prefix = '/placenav_teb_bridge/'
                    self.assertEqual(params[prefix+'reference_mode'], reference)
                    self.assertEqual(params[prefix+'path_timeout'], 1.25)
                    self.assertIs(params[prefix+'TebLocalPlannerROS/shrink_horizon_backup'], False)
                    self.assertEqual(params[prefix+'cmd_vel_topic'], '/fusion_validation/cmd_vel')
                    layers = [p['name'] for p in params[prefix+'local_costmap/plugins']]
                    self.assertEqual(layers, ['fused', 'inflation'] if fusion == 'points'
                                     else ['d435', 'unik3d', 'inflation'])
                    self.assertEqual(sum(n.name == 'placenav_teb_bridge' for n in config.nodes), 1)
                    for gate in ('d435_gate', 'unik3d_gate'):
                        self.assertEqual(sum(n.name == gate for n in config.nodes), 1)
                    snapshots[fusion+'_'+profile+'_'+reference] = params
        if os.environ.get('LAUNCH_OPTIONS_OUTPUT'):
            Path(os.environ['LAUNCH_OPTIONS_OUTPUT']).write_text(json.dumps(snapshots, indent=2)+'\n')

    def test_default_controls_match_original_bridge_launch(self):
        _, params = self.load()
        bridge = ET.parse(ROOT.parent/'placenav_teb_bridge/launch/bridge.launch').getroot()
        defaults = {arg.attrib['name']: arg.attrib.get('default') for arg in bridge.findall('arg')}
        self.assertEqual(params['/placenav_teb_bridge/reference_mode'], defaults['reference_mode'])
        self.assertEqual(params['/placenav_teb_bridge/path_timeout'], float(defaults['path_timeout']))
        self.assertEqual(params['/placenav_teb_bridge/TebLocalPlannerROS/shrink_horizon_backup'],
                         defaults['shrink_horizon_backup'] == 'true')
        self.assertEqual(params['/placenav_teb_bridge/comparison_profile'], 'current')

    def test_frames_reach_gates_tf_batch_maps_teb_and_path_runner(self):
        config, params = self.load('global_frame:=test_odom', 'robot_base_frame:=test_base',
                                   'odom_topic:=/test/odom', 'fusion_mode:=points', 'debug_costmaps:=true')
        for gate in ('d435_gate', 'unik3d_gate'):
            self.assertEqual(params['/obstacles/'+gate+'/fixed_frame'], 'test_odom')
            self.assertEqual(params['/obstacles/'+gate+'/base_frame'], 'test_base')
        for ns in ('/placenav_teb_bridge', '/obstacles/source_costmaps'):
            self.assertEqual(params[ns+'/local_costmap/global_frame'], 'test_odom')
            self.assertEqual(params[ns+'/local_costmap/robot_base_frame'], 'test_base')
            for layer in ('d435', 'unik3d', 'fused'):
                self.assertEqual(params[ns+'/local_costmap/'+layer+'/cloud/sensor_frame'], 'test_odom')
        self.assertEqual(params['/placenav_teb_bridge/TebLocalPlannerROS/map_frame'], 'test_odom')
        self.assertEqual(params['/placenav_teb_bridge/TebLocalPlannerROS/odom_topic'], '/test/odom')
        self.assertEqual(params['/placenav_runner/global_frame'], 'test_odom')
        self.assertEqual(params['/placenav_runner/robot_base_frame'], 'test_base')
        self.assertFalse(any(n.name == 'c2_floor_candidate_tf' for n in config.nodes))
        self.assertEqual(params['/obstacles/unik3d_gate/frame_override'], '')
        self.assertEqual(sum(n.name == 'comparison' for n in config.nodes), 1)

    def test_navigation_tuning_and_recording_parameters(self):
        _, params = self.load('start_node_idx:=5', 'goal_node_idx:=31', 'lookahead:=2', 'nomad_seed:=7',
                              'img_size:=[96,96]', 'recompute_place_recognition_db:=true', 'diagnostics:=true')
        for key, value in dict(start_node_idx=5, goal_node_idx=31, lookahead=2, nomad_seed=7,
                               img_size=[96, 96], recompute_place_recognition_db=True).items():
            self.assertEqual(params['/placenav_runner/'+key], value)
        topics = params['/session_recorder/diagnostic_topics']
        self.assertIn('/placenav_teb_bridge/diagnostics', topics)
        self.assertNotIn('/rwrc26/navigation_trace', topics)
        self.assertTrue(params['/session_recorder/diagnostics'])

    def test_invalid_reference_controls_fail_before_nodes_start(self):
        for argument in ('reference_mode:=typo', 'path_timeout:=-1', 'path_timeout:=nan', 'path_timeout:=inf'):
            with self.subTest(argument=argument), self.assertRaises(Exception):
                self.load(argument)

    def test_explicit_drive_switch(self):
        _, params = self.load('drive_robot:=true', 'robot_cmd_vel_topic:=/test/drive')
        self.assertEqual(params['/placenav_teb_bridge/cmd_vel_topic'], '/test/drive')


if __name__ == '__main__':
    unittest.main()
