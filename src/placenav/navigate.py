import numpy as np
import torch
import yaml
import copy
import argparse
import time

from pathlib import Path
from typing import List, Tuple, Dict
from threading import Lock

# ROS
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray, Time

# Place recognition
from place_recognition.bayesian_querier import PlaceRecognitionTopologicalFilter
from place_recognition.sliding_window_querier import PlaceRecognitionSlidingWindowFilter
from place_recognition.feature_extractor import FeatureExtractor
from place_recognition import extract_database
from place_recognition.vpr_config import image_transform, database_signature
from place_recognition.odometry_motion import OdometryHistory
from placenav.topomap_odometry import OdomSample
from placenav.subgoal_selector import ProgressSubgoalSelector

import parser
from utils import (to_numpy, load_model, read_image, get_image_transform,
                   waypoints_to_ros_msg, bridge, odometry_msg_to_numpy)

# Visualization
from placenav_viz_msgs.msg import Viz, LocalizationDiagnostics

class PlaceNavNode:

    img_suffix = ['png', 'jpg', 'jpeg']

    def __init__(
            self,
            args: argparse.Namespace,
        ):
        self.motion_model = args.motion_model
        if self.motion_model == 'odometry' and not (
                args.topomap_base_dir / args.topomap_dir / 'node_odometry.json').is_file():
            raise FileNotFoundError('Odometry motion requires node_odometry.json; '
                                    'use an odometry map or --motion-model legacy')
        self.odom_max_gap = args.odom_max_gap
        # Set device to cuda if requested and available
        if args.device == "cuda" and torch.cuda.is_available():
            rospy.loginfo("Using CUDA")
            self.device = torch.device("cuda")
        else:
            rospy.loginfo("Using CPU")
            self.device = torch.device("cpu")

        # Get the image preprocessing transform for the neural network models
        with args.model_config_path.open(encoding="utf-8") as f:
            wp_config = yaml.safe_load(f)[args.wp_model]
        args.img_size = args.img_size or wp_config["image_size"]
        if list(args.img_size) != list(wp_config["image_size"]) and args.wp_model != "gnm_large":
            raise ValueError(f"{args.wp_model} requires image_size={wp_config['image_size']}")
        if not 0 <= args.target_waypoint_idx < wp_config["len_traj_pred"]:
            raise ValueError("--target-waypoint-idx is outside the model's predicted trajectory")
        self._nomad_seed = args.nomad_seed
        self._image_transform = get_image_transform(args.img_size)
        if args.wp_model in ("vint", "nomad"):
            from placenav.waypoint_backends import waypoint_transform
            self._image_transform = waypoint_transform(args.img_size)
        self._vpr_transform = self._image_transform

        # Load the topological map
        self._load_topomap(args.topomap_base_dir, args.topomap_dir)

        # Load robot config
        with args.robot_config_path.open(mode="r", encoding="utf-8") as f:
            robot_configs = yaml.safe_load(f)
        self.robot_config = robot_configs[args.robot]
        self.odometry_topic = args.odom_topic or self.robot_config['odom_topic']

        # Initialize place recognition
        if args.subgoal_mode == 'place_recognition':
            self._setup_place_recognition(
                args.model_config_path,
                args.model_weight_dir,
                args.pr_model,
                filter_mode=args.filter_mode,
                transition_model_window_lower=args.transition_model_window_lower,
                transition_model_window_upper=args.transition_model_window_upper,
                bayesian_filter_delta=args.filter_delta,
                recompute_db=args.recompute_place_recognition_db,
                motion_model=self.motion_model,
                odom_noise=args.odom_noise,
                )
            
        # Initialize the waypoint (and temporal distance) model
        self._setup_waypoint_model(
            args.model_config_path,
            args.model_weight_dir,
            args.wp_model,
            args.img_size,
            )
        
        self._setup_ros()

        if args.subgoal_mode == 'place_recognition':
            ready_tmp_path = self._place_recognition_ready_path.with_suffix(
                self._place_recognition_ready_path.suffix + ".tmp"
            )
            ready_tmp_path.write_text(
                f"images={self.map_size}\n",
                encoding="utf-8",
            )
            ready_tmp_path.replace(self._place_recognition_ready_path)

    def _load_topomap(self, topomap_images_base_dir, topomap_dir: Path):
        # List the topomap images with suffix img_suffix in the directory,
        # extract the filenames and sort them        
        self.topomap_img_dir = topomap_images_base_dir / topomap_dir

        topomap_images = []
        for img_suffix in self.img_suffix:
            topomap_images.extend(list(self.topomap_img_dir.glob(f"*.{img_suffix}")))
        topomap_filenames = [img.name for img in topomap_images]
        topomap_filenames = sorted(topomap_filenames, key=lambda filename: int(filename.split(".")[0]))

        # Load the topomap images
        map_size = len(topomap_filenames)
        topomap_images = []
        for i in range(map_size):
            image_path = self.topomap_img_dir / topomap_filenames[i]
            img = read_image(image_path)
            img = self._image_transform(img.astype(np.uint8)).unsqueeze(0)
            topomap_images.append(img)
        
        self.map_size = map_size
        self.topomap_filenames = topomap_filenames
        self.topomap_images = topomap_images

    def _setup_place_recognition(
            self,
            model_config_path: Path,
            model_weight_dir: Path,
            place_recognition_model: str,
            filter_mode: str,
            transition_model_window_lower: int,
            transition_model_window_upper: int,
            bayesian_filter_delta: int,
            recompute_db: bool = False,
            motion_model: str = 'odometry',
            odom_noise: float = 0.1,
        ):
        # Load the place recognition model config
        with model_config_path.open(mode="r", encoding="utf-8") as f:
            confs = yaml.safe_load(f)
            conf = confs[place_recognition_model]
        conf['model']["checkpoint_path"] = model_weight_dir / conf['model']["checkpoint_path"]
        # CosPlace must not change resolution when the waypoint backend changes.
        conf.setdefault('preprocessing', {'resize': [85, 64], 'normalize_first': False})
        self._vpr_transform = image_transform(
            conf['preprocessing']['resize'], conf['preprocessing'].get('normalize_first', False))
        if not conf['model']['checkpoint_path'].is_file():
            raise FileNotFoundError(
                f"Missing {conf['model']['checkpoint_path']}. Prepare the selected VPR model first.")
        signature = database_signature(conf, self.topomap_img_dir, self.topomap_filenames)

        # Extract the global descriptors from the topomap images
        place_recognition_db_path = self.topomap_img_dir / f"global-feats-{place_recognition_model}.h5"
        self._place_recognition_ready_path = Path(f"{place_recognition_db_path}.ready")
        self._place_recognition_ready_path.unlink(missing_ok=True)

        if recompute_db or not extract_database.database_matches_images(
            place_recognition_db_path,
            self.topomap_filenames,
            signature,
        ):
            rospy.loginfo(
                "Place recognition database is missing or incomplete; "
                f"extracting features for {self.map_size} topomap images"
            )
            extract_database.main(
                conf,
                self.topomap_img_dir,
                self._vpr_transform,
                self.topomap_img_dir,
                as_half=False,
                signature=signature,
                image_list=self.topomap_filenames,
                )

        if not extract_database.database_matches_images(
            place_recognition_db_path,
            self.topomap_filenames,
            signature,
        ):
            raise RuntimeError(
                "Place recognition database is incomplete after feature extraction: "
                f"expected {self.map_size} image entries in {place_recognition_db_path}"
            )

        extractor = FeatureExtractor(conf, self.device)

        if filter_mode == 'bayesian':
            self.place_recognition_querier = PlaceRecognitionTopologicalFilter(
                extractor,
                place_recognition_db_path,
                self.topomap_img_dir,
                delta=bayesian_filter_delta,
                window_lower=transition_model_window_lower,
                window_upper=transition_model_window_upper,
                motion_model=motion_model,
                odom_noise=odom_noise,
                )
            
        elif filter_mode == 'sliding_window':
            self.place_recognition_querier = PlaceRecognitionSlidingWindowFilter(
                extractor,
                place_recognition_db_path,
                str(self.topomap_img_dir))
        else:
            raise ValueError(f"Filter mode {filter_mode} not recognized")
        
    def _setup_waypoint_model(
            self,
            model_config_path: Path,
            model_weight_dir: Path,
            model: str,
            img_size: Tuple[int, int],
            ):
        # Load waypoint model config
        with model_config_path.open(mode="r", encoding="utf-8") as f:
            model_configs = yaml.safe_load(f)
        model_config = model_configs[model]
        model_config["img_size"] = img_size
        model_config["seed"] = self._nomad_seed
        self.waypoint_scale = (self.robot_config["max_v"] / self.robot_config["frame_rate"]
                               if model_config.get("metric_scale") == "robot_step" else 1.0)
        rospy.loginfo("Waypoint backend=%s, image=%s, points=%d, XY scale=%g",
                      model, img_size, model_config["len_traj_pred"], self.waypoint_scale)

        model_path = model_weight_dir / model_config["checkpoint_path"]
        if not model_path.exists():
            raise FileNotFoundError(f"Model weights not found at {model_path}")
        rospy.loginfo(f"Loading model weights from {model_path}")

        waypoint_model = load_model(
            model_path,
            model_config,
            self.device)
        waypoint_model.eval()
        self.waypoint_model = waypoint_model

        # Initialize the context queue
        self.context_size = model_config["context_size"]
        self.context_queue: List[Dict[Time, torch.Tensor]] = []

    def _setup_ros(self):
        self.obs_lock = Lock()
        self.odom_lock = Lock()
        self.goal_selector = None
        self._goal_reached = False
        self._terminal_stop_sent = False
        # max_v is a command limit; a manually recorded bag may move faster.
        self.odom_history = OdometryHistory(
            self.odom_max_gap, max_speed=max(3., 3 * self.robot_config['max_v']))
        self._last_localization_stamp = None
        self._last_odom_epoch = None
        if self.motion_model == 'odometry':
            self.odom_sub = rospy.Subscriber(
                self.odometry_topic, Odometry, self.odom_callback, queue_size=100)

        obs_sub = rospy.Subscriber(
            self.robot_config['camera_topic'],
            Image,
            self.obs_callback,
            queue_size=1)
        
        self.waypoint_pub = rospy.Publisher(
            self.robot_config['waypoint_topic'],
            Float32MultiArray,
            queue_size=1)
        
        self.stop_sub = rospy.Subscriber(
            self.robot_config['stop_topic'],
            Bool,
            self.stop_callback,
            queue_size=1)
        
        self.goal_pub = rospy.Publisher(
            self.robot_config['reached_goal_topic'],
            Bool,
            queue_size=1, latch=True)
        self.stop_pub = rospy.Publisher(
            self.robot_config['stop_topic'], Bool, queue_size=1, latch=True)
        
        self.viz_pub = rospy.Publisher(
            self.robot_config['toponav_viz_info_topic'],
            Viz,
            queue_size=1)
        # publish subgoal images
        self.subgoal_pub = rospy.Publisher(
            "/toponav/subgoal_image",
            Image,
            queue_size=1,
            latch=True
        )
        self._last_subgoal_idx = None

        self.localization_diagnostics_pub = rospy.Publisher(
            self.robot_config['localization_diagnostics_topic'],
            LocalizationDiagnostics,
            queue_size=1)

        rospy.init_node("PlaceNavNode", anonymous=False)
        rospy.on_shutdown(self.publish_terminal_stop)
        rospy.loginfo('Localization motion model=%s, odometry=%s',
                      self.motion_model, self.odometry_topic)

        
    def obs_callback(self, msg: Image):
        if self._terminal_stop_sent:
            return
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        vpr_img = self._vpr_transform(img).unsqueeze(0)
        with self.obs_lock:
            if (self.motion_model == 'odometry' and self.context_queue and
                    msg.header.stamp < self.context_queue[-1]['timestamp']):
                self.context_queue.clear()
            img = self._image_transform(img).unsqueeze(0)
            if len(self.context_queue) < self.context_size + 1:
                self.context_queue.append( {'timestamp': msg.header.stamp, 'img': img, 'vpr_img': vpr_img})
            else:            
                self.context_queue.pop(0)
                self.context_queue.append({'timestamp': msg.header.stamp, 'img': img, 'vpr_img': vpr_img})

    def odom_callback(self, msg: Odometry):
        if self._terminal_stop_sent:
            return
        q = msg.pose.pose.orientation
        expected = self.place_recognition_querier.motion.child_frame_id
        if (msg.child_frame_id != expected or
                not np.isfinite([q.x, q.y, q.z, q.w]).all() or
                q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12):
            rospy.logwarn_throttle(5, 'Ignoring invalid odometry or unexpected body frame')
            return
        pose = tuple(float(v) for v in odometry_msg_to_numpy(msg))
        sample = OdomSample(msg.header.stamp.to_nsec(), pose,
                            msg.header.frame_id, msg.child_frame_id)
        try:
            with self.odom_lock:
                reason = self.odom_history.add(sample)
            if reason:
                rospy.logwarn('Odometry continuity reset: %s', reason)
        except ValueError as error:
            rospy.logwarn_throttle(5, str(error))

    def _image_odometry(self, stamp_ns):
        with self.odom_lock:
            sample = self.odom_history.at(stamp_ns)
            epoch = self.odom_history.epoch
        if sample is not None and epoch != self._last_odom_epoch:
            self.place_recognition_querier.reset_odometry()
            if self.goal_selector is not None:
                self.goal_selector.reset_confirmation()
            self._last_odom_epoch = epoch
        return sample

    def stop_callback(self, msg: Bool):
        if msg.data and not self._terminal_stop_sent:
            rospy.loginfo("Stopping...")
            rospy.signal_shutdown("shutdown")

    def publish_terminal_stop(self):
        """Send a terminal event before transports close; use wall time if paused."""
        if self._terminal_stop_sent:
            return
        self._terminal_stop_sent = True
        publisher = self.goal_pub if self._goal_reached else self.stop_pub
        for _ in range(3):
            self.waypoint_pub.publish(Float32MultiArray(data=[0., 0.]))
            publisher.publish(Bool(data=True))
            time.sleep(.05)

    def navigate(self, args: argparse.Namespace):
        rospy.loginfo("Starting navigation")
        
        closest_node_idx = args.start_node_idx
        assert -1 <= args.goal_node_idx < self.map_size, "Invalid goal index"
        if args.goal_node_idx == -1:
            args.goal_node_idx = self.map_size - 1
        if self.motion_model == 'odometry':
            self.goal_selector = ProgressSubgoalSelector(
                self.place_recognition_querier.motion, args.goal_node_idx,
                mode=args.subgoal_update, lookahead=args.lookahead,
                threshold=args.pass_threshold, confirmations=args.pass_confirmations,
                min_local_mass=args.pass_local_mass,
                final_tolerance=args.final_goal_tolerance)

        # Initialize iterative average calculation for runtime
        avg_runtime = 0
        avg_runtime_count = 0

        first_pass = True
        reached_goal = False
        rate = rospy.Rate(self.robot_config['frame_rate'])
        while not rospy.is_shutdown():

            start_time = time.time()

            with self.obs_lock:
                context = copy.deepcopy(self.context_queue)
            
            if len(context) >= self.context_size + 1:

                current_obs = context[-1]['vpr_img'].to(self.device)
                current_obs_timestamp = context[-1]['timestamp']
                odometry = None
                if self.motion_model == 'odometry':
                    stamp_ns = current_obs_timestamp.to_nsec()
                    if stamp_ns == self._last_localization_stamp:
                        rate.sleep()
                        continue
                    if (self._last_localization_stamp is not None and
                            stamp_ns < self._last_localization_stamp):
                        self.place_recognition_querier.reset_odometry()
                        self._last_odom_epoch = None
                    odometry = self._image_odometry(stamp_ns)
                    if odometry is None:
                        rospy.logwarn_throttle(5, 'Waiting for image-time odometry on %s',
                                               self.odometry_topic)
                        rate.sleep()
                        continue
                # Cat the context images along the channel dimension
                context = torch.cat([obs['img'] for obs in context], dim=1).to(self.device)

                if (
                    first_pass and
                    args.subgoal_mode == 'place_recognition' and
                    args.filter_mode == 'bayesian'
                    ):
                    # Initialize the belief distribution of the
                    # Bayesian filter prior to first query
                    self.place_recognition_querier.initialize_model(current_obs)
                    first_pass = False
                
                if args.subgoal_mode == 'place_recognition':
                    
                    # Place recognition with Bayesian filter
                    if args.filter_mode == 'bayesian':
                        if self.motion_model == 'odometry':
                            closest_node_idx, score = self.place_recognition_querier.match(
                                current_obs, odometry)
                            self._last_localization_stamp = current_obs_timestamp.to_nsec()
                        else:
                            closest_node_idx, score = self.place_recognition_querier.match(current_obs)

                    # Place recognition with sliding window filter
                    elif args.filter_mode == 'sliding_window':
                        start = max(closest_node_idx - args.window_radius, 0)
                        end = min(closest_node_idx + args.window_radius +1, args.goal_node_idx +1)
                        closest_node_idx = self.place_recognition_querier.match(current_obs, start, end)

                    else:
                        raise ValueError(f"Filter mode {args.filter_mode} not recognized for subgoal mode {args.subgoal_mode}")
                    
                    # How many nodes ahead from closest node to choose the subgoal
                    if self.goal_selector is not None:
                        previous_goal = self.goal_selector.subgoal
                        subgoal_idx, reached_goal = self.goal_selector.update(
                            closest_node_idx, current_obs_timestamp.to_nsec())
                        if previous_goal != subgoal_idx:
                            rospy.loginfo('Subgoal %s -> %s (%s, %s)', previous_goal,
                                          subgoal_idx, self.goal_selector.status,
                                          self.goal_selector.probabilities)
                        if self.goal_selector.status == 'REMOTE_BELIEF':
                            rospy.logwarn_throttle(5, 'Final goal held: belief outside goal neighborhood')
                    else:
                        subgoal_idx = min(closest_node_idx + args.lookahead, args.goal_node_idx)
                        reached_goal = closest_node_idx == args.goal_node_idx
                    sg_img = self.topomap_images[subgoal_idx].to(self.device)

                    # Predict the waypoints to subgoal
                    if reached_goal:
                        waypoints = np.empty((0, 2))
                    else:
                        with torch.no_grad():
                            _, waypoints = self.waypoint_model(context, sg_img)
                        waypoints = to_numpy(waypoints[0])

                elif args.subgoal_mode == 'temporal_distance':
                    if args.filter_mode == 'bayesian':
                        raise NotImplementedError("Temporal distance subgoal mode cannot be used with the Bayesian filter")

                    # The regular GNM approach
                    elif args.filter_mode == 'sliding_window':
                        start = max(closest_node_idx - args.window_radius, 0)
                        end = min(closest_node_idx + args.window_radius +1, args.goal_node_idx +1)

                        distances = []
                        waypoints_candidates = []      
                        for sg_idx in range(start, end):
                            sg_img = self.topomap_images[sg_idx].to(self.device)
                            with torch.no_grad():
                                    dist, waypoint = self.waypoint_model(context, sg_img)

                            distances.append(to_numpy(dist[0]))
                            waypoints_candidates.append(to_numpy(waypoint[0]))

                        # look for closest node
                        closest_node = np.argmin(distances)

                        # choose subgoal and output waypoints
                        if distances[closest_node] > args.close_threshold:
                            sg_idx = closest_node
                        else:
                            sg_idx = min(closest_node + 1, len(waypoints_candidates) - 1)

                        waypoints = waypoints_candidates[sg_idx]
                            
                        # 'closest_node' is the index of the subgoal in the window,
                        # so we need to add 'start' to get the index in the topomap
                        closest_node_idx = start + closest_node
                        subgoal_idx = start + sg_idx
                        reached_goal = closest_node_idx == args.goal_node_idx
                    else:
                        raise ValueError(f"Filter mode {args.filter_mode} not recognized for subgoal mode {args.subgoal_mode}")
                else:
                    raise ValueError(f"Subgoal mode {args.subgoal_mode} not recognized")

                if not np.isfinite(waypoints).all():
                    rospy.logerr_throttle(2.0, "Non-finite waypoint prediction; not publishing path")
                    rate.sleep()
                    continue
                # Official ViNT/NoMaD actions are normalized displacements, not
                # meters. Preserve legacy GNM behavior; scale only configured models.
                waypoints[:, :2] *= self.waypoint_scale

                # Publish the waypoint
                if not reached_goal:
                    chosen_waypoint = waypoints[args.target_waypoint_idx]
                    waypoint_msg = Float32MultiArray()
                    waypoint_msg.data = chosen_waypoint
                    self.waypoint_pub.publish(waypoint_msg)

                # Update the average runtime
                loop_duration = time.time() - start_time
                avg_runtime = (avg_runtime * avg_runtime_count + loop_duration) / (avg_runtime_count + 1)
                avg_runtime_count += 1
                rospy.loginfo(f"Average runtime: {avg_runtime:.3f} s, Runtime: {loop_duration:.3f} s")

                # Publish information for visualization
                viz_msg = Viz()
                viz_msg.query_timestamp = current_obs_timestamp
                viz_msg.subgoal_idx = subgoal_idx
                viz_msg.waypoints = waypoints_to_ros_msg(waypoints)

                diagnostics = getattr(getattr(self, 'place_recognition_querier', None), "last_diagnostics", None)
                if diagnostics is not None:
                    diagnostics_msg = LocalizationDiagnostics()
                    diagnostics_msg.query_timestamp = current_obs_timestamp
                    diagnostics_msg.current_node_idx = closest_node_idx
                    diagnostics_msg.subgoal_idx = subgoal_idx
                    diagnostics_msg.cosplace_top1_node_idx = diagnostics["cosplace_top1_node_idx"]
                    diagnostics_msg.cosplace_top2_node_idx = diagnostics["cosplace_top2_node_idx"]
                    diagnostics_msg.cosplace_top1_score = diagnostics["cosplace_top1_score"]
                    diagnostics_msg.cosplace_top2_score = diagnostics["cosplace_top2_score"]
                    diagnostics_msg.cosplace_score_margin = diagnostics["cosplace_score_margin"]
                    diagnostics_msg.bayesian_belief_max = diagnostics["bayesian_belief_max"]
                    diagnostics_msg.bayesian_belief_std = diagnostics["bayesian_belief_std"]
                    diagnostics_msg.bayesian_belief_entropy = diagnostics["bayesian_belief_entropy"]
                    self.localization_diagnostics_pub.publish(diagnostics_msg)
                self.viz_pub.publish(viz_msg)

                # subgoal image publishing
                if subgoal_idx != self._last_subgoal_idx:
                    img_path = self.topomap_img_dir / self.topomap_filenames[subgoal_idx]
                    img = read_image(img_path)

                    img_msg = bridge.cv2_to_imgmsg(img, encoding="rgb8")
                    img_msg.header.stamp = current_obs_timestamp
                    self.subgoal_pub.publish(img_msg)

                    self._last_subgoal_idx = subgoal_idx

                if reached_goal:
                    self._goal_reached = True
                    self.publish_terminal_stop()
                    rospy.loginfo("Reached goal. Stopping...")
                    return
                self.goal_pub.publish(False)
                
                rate.sleep()

if __name__ == '__main__':
    args = parser.parse_args()

    placenav_node = PlaceNavNode(args)
    try:
        placenav_node.navigate(args)
        if placenav_node._goal_reached and not rospy.is_shutdown():
            # Keep the terminal latch available to late/restarted bridge subscribers.
            rospy.spin()
    finally:
        placenav_node.publish_terminal_stop()
