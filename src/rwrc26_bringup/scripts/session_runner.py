#!/usr/bin/env python3
"""ROS launch adapters for the existing navigation script and dated bag recording.

Only main() imports rospy so command construction and process cleanup can be
tested without ROS, model loading, a camera, or robot command publishers.
"""
from datetime import datetime, timedelta, timezone
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
import uuid


def supports_navigation_trace(directory):
    """Inspect the selected checkout without importing ROS, Torch or models."""
    parser = Path(directory)/'placenav/parser.py'
    if not parser.is_file():
        return False
    tree = ast.parse(parser.read_text(encoding='utf-8'))
    return any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
               and node.func.attr == 'add_argument'
               and any(isinstance(arg, ast.Constant) and arg.value == '--trace-navigation'
                       for arg in node.args) for node in ast.walk(tree))


def navigation_command(params):
    directory = Path(params['placenav_src']).expanduser().resolve()
    script = directory/'navigate_whill.sh'
    if not script.is_file():
        raise ValueError('navigate_whill.sh not found: '+str(script))
    radius = int(params.get('window_radius', 3))
    if radius < 1:
        raise ValueError('window_radius must be positive')
    command = ['bash', str(script),
               '--topomap-dir', str(params['topomap_dir']),
               '--wp-model', str(params.get('wp_model', 'nomad')),
               '--pr-model', str(params.get('pr_model', 'salad')),
               '--subgoal-mode', str(params.get('subgoal_mode', 'place_recognition')),
               '--filter-mode', str(params.get('filter_mode', 'bayesian')),
               '--window-radius', str(radius)]
    integers = {'start_node_idx': 0, 'goal_node_idx': -1, 'lookahead': 1,
                'close_threshold': 3, 'target_waypoint_idx': 2, 'nomad_seed': 0,
                'transition_model_window_lower': -1, 'transition_model_window_upper': 2,
                'filter_delta': 10}
    for name, default in integers.items():
        command += ['--'+name.replace('_', '-'), str(int(params.get(name, default)))]
    command += ['--device', str(params.get('device', 'cuda'))]
    motion = str(params.get('motion_model', 'odometry'))
    if motion not in ('legacy', 'odometry'):
        raise ValueError('motion_model must be legacy or odometry')
    gap = float(params.get('odom_max_gap', 0.2))
    noise = float(params.get('odom_noise', 0.1))
    if not math.isfinite(gap) or gap <= 0 or not math.isfinite(noise) or noise < 0:
        raise ValueError('odom_max_gap must be positive and odom_noise nonnegative')
    command += ['--motion-model', motion,
                '--odom-topic', str(params.get('odom_topic', '/odom')),
                '--odom-max-gap', str(gap), '--odom-noise', str(noise)]
    size = params.get('img_size', [])
    if not isinstance(size, list) or (size and (len(size) != 2 or any(
            type(value) is not int or value <= 0 for value in size))):
        raise ValueError('img_size must be [] or [positive width, positive height]')
    if size:
        command += ['--img-size']+[str(value) for value in size]
    if params.get('recompute_place_recognition_db', False):
        command.append('--recompute-place-recognition-db')
    command += ['--path-adapter-source-frame', str(params.get('robot_base_frame', 'base_link')),
                '--path-adapter-target-frame', str(params.get('global_frame', 'odom'))]
    if not params.get('visualization', True):
        command.append('--no-visualization')
    if params.get('diagnostics', False) and supports_navigation_trace(directory):
        command.append('--trace-navigation')
    # navigate_whill.sh starts the GNM path adapter itself. Do not duplicate it.
    return command, directory


def recording_command(params, now=None):
    requested = Path(params['bag_dir']).expanduser()
    directory = requested.resolve()
    # Both a direct protected path and a symlink into the protected data tree
    # must be rejected, even when this package is copied to the robot workspace.
    if 'tsukuba' in requested.absolute().parts or 'tsukuba' in directory.parts:
        raise ValueError('Recording inside tsukuba is prohibited')
    prefix = str(params.get('bag_prefix', ''))
    if not re.fullmatch(r'[A-Za-z0-9_-]*', prefix):
        raise ValueError('bag_prefix must be empty or contain only letters, digits, _ and -')
    topics = list(dict.fromkeys(params['topics'] + params.get('diagnostic_topics', []) + params.get('comparison_topics', [])))
    if params.get('diagnostics', False) and supports_navigation_trace(params.get('placenav_src', '')):
        topics = list(dict.fromkeys(topics + ['/rwrc26/navigation_metadata', '/rwrc26/navigation_trace',
                                             '/rwrc26/path_adapter_trace', '/rwrc26/teb_trace']))
    if not topics or any(not isinstance(t, str) or not t.startswith('/') or any(c.isspace() for c in t) for t in topics):
        raise ValueError('topics must be a nonempty list of absolute ROS topic names')
    buffer_mb = int(params.get('buffer_mb', 512))
    if buffer_mb < 1:
        raise ValueError('buffer_mb must be positive')
    now = now or datetime.now(timezone(timedelta(hours=9)))
    if now.tzinfo is None:
        raise ValueError('Recording timestamp must include a timezone')
    timestamp = now.astimezone(timezone(timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f')
    name = prefix+'_'+timestamp+'_'+uuid.uuid4().hex[:8] if prefix else timestamp
    bag = directory/(name+'.bag')
    metadata = directory/(name+'.session.json')
    # An exclusive metadata reservation protects older recordings, including
    # timestamp-only names. rosbag creates/finalizes its own .active file.
    if bag.exists() or Path(str(bag)+'.active').exists() or metadata.exists():
        raise ValueError('Recording path already exists')
    return ['rosbag', 'record', '--lz4', '--buffsize', str(buffer_mb), '-O', str(bag)]+topics, directory, metadata


def stop_process(process):
    """Stop only the process group created by this runner, including children."""
    for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        try:
            # Do this even when the script exited but its children remain.
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        # The group can still contain descendants after the group leader exits.
        deadline = time.monotonic()+min(timeout, .5)
        while time.monotonic() < deadline:
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(.025)
    process.wait()


def package_fingerprints(placenav_src=None):
    """Hash deployed configuration, code and actual installed executables."""
    import rospkg
    import roslib.packages
    packages = rospkg.RosPack()
    hashes = {}
    executables = {'source_costmaps': ('cloud_gate_node', 'source_costmaps_node', 'cloud_batch.py'),
                   'rwrc26_bringup': ('session_runner.py',),
                   'placenav_teb_bridge': ('placenav_teb_bridge_node',)}
    for name in executables:
        root = Path(packages.get_path(name))
        paths = [p for folder in ('config', 'launch', 'scripts', 'src', 'include')
                 for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts]
        paths += [p for p in (root/'package.xml', root/'CMakeLists.txt') if p.is_file()]
        if placenav_src:
            selected = Path(placenav_src).expanduser().resolve()
            if name == 'placenav_teb_bridge':
                paths += [p for folder in ('src', 'config', 'launch')
                          for p in (selected/name/folder).rglob('*') if p.is_file()]
            elif name == 'rwrc26_bringup':
                paths += [selected/relative for relative in (
                    'navigate_whill.sh', 'run_gnm_path_adapter_whill.sh', 'placenav/parser.py',
                    'placenav/navigate.py', 'placenav/gnm_path_adapter.py', 'config/robots.yaml',
                    'config/models.yaml') if (selected/relative).is_file()]
        for executable in executables[name]:
            paths += [Path(p) for p in (roslib.packages.find_node(name, executable) or [])]
        if name == 'source_costmaps':
            paths += [p.parent.parent/'libsource_costmaps_cloud.so' for p in list(paths)
                      if (p.parent.parent/'libsource_costmaps_cloud.so').is_file()]
        for path in sorted(set(paths)):
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(block)
            hashes[str(path)] = digest.hexdigest()
    return hashes


def main():
    import rospy
    rospy.init_node('session_runner')
    params = rospy.get_param('~')
    role = params.get('role')
    try:
        if role == 'navigation':
            command, directory = navigation_command(params)
            if params.get('diagnostics', False) and '--trace-navigation' not in command:
                rospy.logwarn('Selected PlaceNav has no --trace-navigation support; recording available ROS diagnostics only.')
        elif role == 'record':
            command, directory, metadata = recording_command(params)
            directory.mkdir(parents=True, exist_ok=True)
            saved = {'timestamp_timezone': 'Asia/Tokyo', 'command': command, 'parameters': {},
                     'sha256': package_fingerprints(params.get('placenav_src'))}
            for key in ('/placenav_runner', '/unidepth_ros_adapter', '/obstacles/d435_gate',
                        '/obstacles/unik3d_gate', '/obstacles/comparison', '/placenav_teb_bridge'):
                saved['parameters'][key] = rospy.get_param(key, {})
            with metadata.open('x') as stream:
                json.dump(saved, stream, indent=2); stream.write('\n')
            rospy.loginfo('Recording bag: %s', command[command.index('-O')+1])
            rospy.loginfo('Session settings: %s', metadata)
        else:
            raise ValueError('role must be navigation or record')
    except (KeyError, TypeError, ValueError, OSError, SyntaxError) as error:
        rospy.logfatal('Session configuration: %s', error)
        return 1
    rospy.loginfo('Starting %s in %s', role, directory)
    try:
        process = subprocess.Popen(command, cwd=str(directory), start_new_session=True)
    except OSError as error:
        rospy.logfatal('Could not start %s: %s', role, error)
        return 1
    lock = threading.Lock()
    def cleanup():
        with lock:
            stop_process(process)
    rospy.on_shutdown(cleanup)
    try:
        while not rospy.is_shutdown():
            status = process.poll()
            if status is not None:
                if status:
                    rospy.logerr('%s exited with status %s', role, status)
                else:
                    rospy.loginfo('%s finished', role)
                return 0 if status == 0 else 1
            time.sleep(.1)  # Wall time: shutdown works even when ROS time pauses.
        return 0
    finally:
        cleanup()


if __name__ == '__main__':
    raise SystemExit(main())
