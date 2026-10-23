'''
Argument parser for the placenav node
(navigate.py)
'''

import argparse
from pathlib import Path

def parse_args():
    parser = argparse.ArgumentParser(
        description=f"PlaceNav node for topological image navigation"
    )
    parser.add_argument(
        "--robot",
        type=str,
        help="robot name",
        required=True,
    )
    parser.add_argument(
        "--img-size",
        "-i",
        default=None,
        type=int,
        nargs=2,
        help="waypoint image size; default is selected model's image_size (not VPR size)",
    )
    parser.add_argument(
        "--topomap-dir",
        "-d",
        type=str,
        help="name of the topomap directory",
        required=True,
    )
    parser.add_argument(
        "--wp-model",
        default="gnm_large",
        type=str,
        choices=["gnm_large", "vint", "nomad"],
        help="waypoint model (default: gnm_large); prepare ViNT/NoMaD weights first",
    )
    parser.add_argument(
        "--pr-model",
        default="cosplace",
        type=str,
        choices=["cosplace", "megaloc", "salad", "anyloc"],
        help="VPR backend (prepare new models with prepare_vpr.py first; default: cosplace)",
    )
    parser.add_argument("--nomad-seed", type=int, default=0,
                        help="per-inference NoMaD diffusion seed (default: 0)")
    parser.add_argument("--nomad-selection", choices=["single", "costmap"], default="single",
                        help="single preserves baseline; costmap selects among goal-conditioned candidates")
    parser.add_argument("--nomad-candidates", type=int, default=8)
    parser.add_argument("--nomad-costmap-topic",
                        default="/placenav_teb_bridge/local_costmap/costmap")
    parser.add_argument("--nomad-base-frame", default="base_link")
    parser.add_argument("--nomad-clearance", type=float, default=.10,
                        help="extra footprint margin in metres (not inflation radius)")
    parser.add_argument("--nomad-continuity-weight", type=float, default=1.)
    parser.add_argument("--nomad-input-timeout", type=float, default=1.)
    parser.add_argument(
        "--close-threshold",
        "-t",
        default=3,
        type=int,
        help="""temporal distance within the next node in the topomap before 
        localizing to it (default: 3)""",
    )
    parser.add_argument(
        "--window-radius",
        "-r",
        default=2,
        type=int,
        help="""temporal number of topomap nodes to look at in the topopmap for
        localization when using sliding window filter (default: 2)""",
    )
    parser.add_argument(
        "--target-waypoint-idx",
        "-w",
        default=2, # close waypoints exihibit straight line motion (the middle waypoint is a good default)
        type=int,
        help=f"""index of the waypoint used for navigation (between 0 and 4 or 
        how many waypoints your model predicts) (default: 2)""",
    )
    parser.add_argument(
        "--goal-node-idx",
        "-g",
        default=-1,
        type=int,
        help="""goal node index in the topomap (if -1, then the goal node is 
        the last node in the topomap) (default: -1)""",
    )
    parser.add_argument(
        "--start-node-idx",
        "-s",
        default=0,
        type=int,
        help="""start node index in the topomap (if 0, then the start node is 
        the first node in the topomap) (default: 0)""",
    )
    parser.add_argument(
        "--filter-mode",
        default="sliding_window",
        type=str,
        help="""Filter mode for choosing the subgoal (sliding_window: GNM default or
         topological_filter: from https://arxiv.org/pdf/2105.03091.pdf) (default: sliding_window)""",
    )
    parser.add_argument(
        "--subgoal-mode",
        default='place_recognition',
        type=str,
        help="""If the subgoal should be chosen using GNM image temporal distance prediction
         or place recognition (place_recognition | temporal_distance) (default: place_recognition)"""
    )
    parser.add_argument(
        "--lookahead",
        default=1,
        type=int,
        help="""Number of nodes ahead from current node to consider for subgoal
            when choosing subgoals with place recognition (default: 1)""",
    )
    parser.add_argument(
        "--recompute-place-recognition-db",
        action="store_true",
        default=False
    )
    parser.add_argument(
        "--device",
        default="cuda",
        type=str,
        help="Device to use for inference (cuda | cpu) (default: cuda)",
    )

    # Bayesian filter params
    parser.add_argument(
        "--transition-model-window-lower",
        default=-1,
        type=int,
        help="""Lower window of the transition model (default: -1)""",
    )
    parser.add_argument(
        "--transition-model-window-upper",
        default=2,
        type=int,
        help="""Upper window of the transition model (default: 2)""",
    )
    parser.add_argument(
        "--filter-delta",
        default=10,
        type=int,
        help="""Filter delta (default: 10)""",
    )

    # Config file paths
    parser.add_argument(
        "--robot-config-path",
        type=Path,
        help="path to config of the robot to control",
        required=True,
    )
    parser.add_argument(
        "--model-config-path",
        type=Path,
        help="path to config of the models",
        required=True,
    )
    parser.add_argument(
        "--topomap-base-dir",
        type=Path,
        help="path to topomap directory",
        required=True,
    )
    parser.add_argument(
        "--model-weight-dir",
        type=Path,
        help="path to model weights directory",
        required=True,
    )


    args, _ = parser.parse_known_args()
    import math
    if not 1 <= args.nomad_candidates <= 32:
        parser.error("--nomad-candidates must be 1..32")
    for name in ("nomad_clearance", "nomad_continuity_weight", "nomad_input_timeout"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) < 0:
            parser.error(name + " must be finite and nonnegative")
    if args.nomad_input_timeout <= 0:
        parser.error("--nomad-input-timeout must be positive")
    if args.nomad_selection == "costmap":
        if args.wp_model != "nomad" or args.subgoal_mode != "place_recognition":
            parser.error("costmap selection requires NoMaD and place_recognition subgoals")
        if args.nomad_candidates < 2:
            parser.error("costmap selection requires at least two candidates")
    return args
