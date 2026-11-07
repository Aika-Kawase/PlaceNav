#!/usr/bin/env python3
"""No cameras, model inference or robot command publishers are used here."""
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/session_runner.py'
spec = importlib.util.spec_from_file_location('session_runner', SCRIPT)
runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)


class SessionRunnerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='rwrc26-session-')
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_navigation_keeps_argument_boundaries_and_working_directory(self):
        directory = self.root/'PlaceNav source'; directory.mkdir()
        (directory/'navigate_whill.sh').write_text('''#!/bin/bash
python3 - "$@" <<'PY'
import json,os,sys
open('captured.json','w').write(json.dumps({'cwd':os.getcwd(),'args':sys.argv[1:]}))
PY
''')
        route = 'route with spaces; touch unexpected_file'
        command, cwd = runner.navigation_command({'placenav_src': str(directory), 'topomap_dir': route,
                                                 'visualization': False, 'window_radius': 3})
        subprocess.run(command, cwd=cwd, check=True)
        captured = json.loads((directory/'captured.json').read_text())
        self.assertEqual(captured['cwd'], str(directory))
        self.assertEqual(captured['args'][captured['args'].index('--topomap-dir')+1], route)
        self.assertIn('--no-visualization', captured['args'])
        self.assertEqual(captured['args'][captured['args'].index('--wp-model')+1], 'nomad')
        self.assertEqual(captured['args'][captured['args'].index('--pr-model')+1], 'salad')
        self.assertEqual(captured['args'][captured['args'].index('--transition-model-window-lower')+1], '0')
        self.assertEqual(captured['args'][captured['args'].index('--transition-model-window-upper')+1], '2')
        self.assertFalse((directory/'unexpected_file').exists())
        self.assertFalse(any(a.startswith('__') for a in captured['args']))

    def test_missing_navigation_script_rejected(self):
        with self.assertRaises(ValueError):
            runner.navigation_command({'placenav_src': str(self.root), 'topomap_dir': 'route'})

    def test_diagnostics_are_opt_in_and_recorded_without_losing_original_topics(self):
        directory=self.root/'PlaceNav'; directory.mkdir(); (directory/'navigate_whill.sh').touch()
        (directory/'placenav').mkdir()
        (directory/'placenav/parser.py').write_text("parser.add_argument('--trace-navigation', action='store_true')")
        command,_=runner.navigation_command(dict(placenav_src=str(directory),topomap_dir='route',diagnostics=True))
        self.assertIn('--trace-navigation',command)
        params=self.record_params(); params['diagnostic_topics']=['/rwrc26/navigation_trace','/tf']
        command,_,_=runner.recording_command(params)
        topics=command[command.index('-O')+2:]
        self.assertEqual(topics,['/tf','/odom','/rwrc26/navigation_trace'])
        params['comparison_topics']=['/obstacles/comparison/batch_trace','/tf']
        command,_,_=runner.recording_command(params)
        self.assertEqual(command[command.index('-O')+2:],
                         ['/tf','/odom','/rwrc26/navigation_trace','/obstacles/comparison/batch_trace'])

    def test_latest_checkout_receives_navigation_options_and_no_unsupported_trace(self):
        directory = Path(os.environ.get('PLACENAV_TEST_SRC', Path(__file__).resolve().parents[2]))
        spec = importlib.util.spec_from_file_location('selected_placenav_parser', directory/'placenav/parser.py')
        parser = importlib.util.module_from_spec(spec); spec.loader.exec_module(parser)
        params = dict(placenav_src=str(directory), topomap_dir='route with spaces',
                      start_node_idx=4, goal_node_idx=19, lookahead=2, nomad_seed=42,
                      close_threshold=5, target_waypoint_idx=6, device='cpu',
                      transition_model_window_lower=-2, transition_model_window_upper=4,
                      filter_delta=12, recompute_place_recognition_db=True, img_size=[96, 96],
                      global_frame='test_odom', robot_base_frame='test_base', diagnostics=True)
        command, _ = runner.navigation_command(params)
        self.assertNotIn('--trace-navigation', command)
        for option, expected in (('--path-adapter-source-frame', 'test_base'),
                                 ('--path-adapter-target-frame', 'test_odom')):
            self.assertEqual(command[command.index(option)+1], expected)
        required = ['--robot', 'whill', '--robot-config-path', 'robots.yaml',
                    '--model-config-path', 'models.yaml', '--topomap-base-dir', 'maps',
                    '--model-weight-dir', 'weights']
        with patch.object(sys, 'argv', ['navigate.py']+required+command[2:]):
            args = parser.parse_args()
        for key in ('topomap_dir', 'start_node_idx', 'goal_node_idx', 'lookahead', 'nomad_seed',
                    'close_threshold', 'target_waypoint_idx', 'device', 'transition_model_window_lower',
                    'transition_model_window_upper', 'filter_delta', 'recompute_place_recognition_db', 'img_size'):
            self.assertEqual(getattr(args, key), params[key], key)

    def test_trace_capability_and_recording_follow_selected_checkout(self):
        directory = self.root/'PlaceNav'; (directory/'placenav').mkdir(parents=True)
        parser = directory/'placenav/parser.py'
        parser.write_text("# --trace-navigation is not supported\nparser.add_argument('--device')")
        self.assertFalse(runner.supports_navigation_trace(directory))
        params = self.record_params(); params.update(placenav_src=str(directory), diagnostics=True)
        command, _, _ = runner.recording_command(params)
        self.assertNotIn('/rwrc26/navigation_trace', command)
        parser.write_text("parser.add_argument('--trace-navigation', action='store_true')")
        command, _, _ = runner.recording_command(params)
        self.assertIn('/rwrc26/navigation_trace', command)

    def test_invalid_image_size_rejected_before_starting_navigation(self):
        directory = self.root/'PlaceNav'; directory.mkdir(); (directory/'navigate_whill.sh').touch()
        for size in ('96,96', [96], [96, 0], [96.0, 96], [True, 96]):
            with self.assertRaises(ValueError):
                runner.navigation_command(dict(placenav_src=str(directory), topomap_dir='route', img_size=size))

    def test_whill_script_routes_frames_to_adapter_only(self):
        source = Path(os.environ.get('PLACENAV_TEST_SRC', Path(__file__).resolve().parents[2]))
        directory = self.root/'PlaceNav'; directory.mkdir()
        (directory/'navigate_whill.sh').write_text((source/'navigate_whill.sh').read_text())
        fake_bin = directory/'bin'; fake_bin.mkdir()
        python = fake_bin/'python3'
        python.write_text('#!'+sys.executable+'''\nimport json,sys,time
from pathlib import Path
Path('nav.json').write_text(json.dumps(sys.argv[1:]))
deadline=time.monotonic()+3
while not Path('adapter.json').exists() and time.monotonic()<deadline: time.sleep(.01)
''')
        python.chmod(0o755)
        adapter = directory/'run_gnm_path_adapter_whill.sh'
        adapter.write_text('#!'+sys.executable+'''\nimport json,sys
from pathlib import Path
Path('adapter.json').write_text(json.dumps(sys.argv[1:]))
''')
        adapter.chmod(0o755)
        command, cwd = runner.navigation_command(dict(placenav_src=str(directory), topomap_dir='route',
                                                      global_frame='test_odom', robot_base_frame='test_base',
                                                      visualization=False))
        env = dict(os.environ, PATH=str(fake_bin)+os.pathsep+os.environ['PATH'])
        subprocess.run(command, cwd=cwd, env=env, check=True, timeout=6)
        nav = json.loads((directory/'nav.json').read_text())
        args = json.loads((directory/'adapter.json').read_text())
        self.assertNotIn('--path-adapter-target-frame', nav)
        self.assertEqual(args[args.index('--source-frame')+1], 'test_base')
        self.assertEqual(args[args.index('--target-frame')+1], 'test_odom')
        self.assertEqual(args[args.index('--expected-waypoints')+1], '8')

    def record_params(self, directory=None):
        return {'bag_dir': str(directory or self.root/'bags'), 'topics': ['/tf', '/tf', '/odom'],
                'cmd_vel_topic': '/local_path/cmd_vel'}

    def test_recording_is_unique_dated_jst_and_keeps_only_requested_topics(self):
        when = datetime(2026, 10, 29, 9, 30, 40, 123456, tzinfo=timezone.utc)
        params = self.record_params(); params['bag_prefix'] = 'placenav_unik3d_teb_tsukuba'
        first, directory, metadata = runner.recording_command(params, when)
        second, _, _ = runner.recording_command(params, when)
        name = Path(first[first.index('-O')+1]).name
        self.assertIn('_20261029_183040_123456_', name)
        self.assertNotEqual(first[first.index('-O')+1], second[second.index('-O')+1])
        self.assertEqual(first.count('/tf'), 1)
        self.assertEqual(first[first.index('-O')+2:], ['/tf', '/odom'])
        self.assertIn('--lz4', first)
        self.assertTrue(metadata.name.endswith('.session.json'))
        self.assertFalse(directory.exists())  # Validation itself never starts recording.

    def test_protected_data_path_and_symlink_are_rejected(self):
        protected = self.root/'tsukuba'; protected.mkdir()
        link = self.root/'looks_safe'; link.symlink_to(protected, target_is_directory=True)
        for directory in (protected/'new', link/'new'):
            with self.assertRaises(ValueError):
                runner.recording_command(self.record_params(directory))
        self.assertFalse((protected/'new').exists())

    def test_empty_prefix_uses_only_jst_timestamp_and_rejects_existing_recordings(self):
        params = self.record_params(); params['bag_prefix'] = ''
        when = datetime(2026, 10, 29, 9, 30, 40, 123456, tzinfo=timezone.utc)
        command, directory, metadata = runner.recording_command(params, when)
        bag = Path(command[command.index('-O')+1])
        self.assertEqual(bag.name, '20261029_183040_123456.bag')
        self.assertEqual(metadata.name, '20261029_183040_123456.session.json')
        del params['bag_prefix']
        default_command, _, _ = runner.recording_command(params, when)
        self.assertEqual(command, default_command)
        directory.mkdir()
        for existing in (bag, Path(str(bag)+'.active'), metadata):
            existing.write_text('keep')
            with self.assertRaises(ValueError):
                runner.recording_command(params, when)
            self.assertEqual(existing.read_text(), 'keep')
            existing.unlink()

    def test_invalid_recording_prefix_and_topic_are_rejected(self):
        for prefix in ('../replace_existing', ' ', 'my bag'):
            params = self.record_params(); params['bag_prefix'] = prefix
            with self.assertRaises(ValueError): runner.recording_command(params)
        params = self.record_params(); params['topics'] = ['--all']
        with self.assertRaises(ValueError): runner.recording_command(params)

    def assert_not_running(self, pid):
        path = Path('/proc')/str(pid)/'stat'
        self.assertTrue(not path.exists() or path.read_text().split()[2] == 'Z')

    def test_cleanup_stops_descendants_after_script_exits(self):
        child_pid = self.root/'child.pid'
        code = '''import subprocess,sys,time
from pathlib import Path
ready=sys.argv[1]+'.ready'
child='import signal,time,sys; from pathlib import Path; signal.signal(signal.SIGINT,signal.SIG_IGN); Path(sys.argv[1]).write_text("ready"); time.sleep(300)'
p=subprocess.Popen([sys.executable,'-c',child,ready])
deadline=time.monotonic()+4
while not Path(ready).exists():
 if time.monotonic()>deadline: raise RuntimeError('child did not start')
 time.sleep(.01)
open(sys.argv[1],'w').write(str(p.pid))
'''
        process = subprocess.Popen(['python3', '-c', code, str(child_pid)], start_new_session=True)
        try:
            process.wait(timeout=5)
            self.assertTrue(child_pid.exists())
            runner.stop_process(process)
            self.assert_not_running(int(child_pid.read_text()))
        finally:
            runner.stop_process(process)


if __name__ == '__main__': unittest.main()
