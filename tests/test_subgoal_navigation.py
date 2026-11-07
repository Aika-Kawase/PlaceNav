"""Exercise the real navigation loop with recorded-style VPR jumps.

Only the navigate method is loaded, avoiding ROS/model import and initialization.
Torch tensors are real; ROS, VPR and waypoint inference are replaced with fakes.
No ROS master, cameras or robot command publishers are started.
"""
import argparse
import ast
import copy
from pathlib import Path
from threading import Lock
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
import torch


SOURCE = Path(__file__).resolve().parents[1] / "src/placenav/navigate.py"


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class NavigationSubgoalTests(unittest.TestCase):
    def run_navigation(self, estimates, *, filter_mode="bayesian", lookahead=1,
                       goal=9, start=0, invalid_frames=(), node=None):
        state = SimpleNamespace(frame=0)
        args = argparse.Namespace(
            start_node_idx=start, goal_node_idx=goal, filter_mode=filter_mode,
            subgoal_mode="place_recognition", lookahead=lookahead,
            window_radius=3, target_waypoint_idx=2,
        )
        querier = SimpleNamespace(initialize_model=Mock(), last_diagnostics=None)

        def match(image, *window):
            current = estimates[state.frame]
            querier.last_diagnostics = dict(
                cosplace_top1_node_idx=current, cosplace_top2_node_idx=current,
                cosplace_top1_score=.9, cosplace_top2_score=.8, cosplace_score_margin=.1,
                bayesian_belief_max=.9, bayesian_belief_std=.1, bayesian_belief_entropy=.1,
            )
            return (current, .9) if filter_mode == "bayesian" else current

        querier.match = Mock(side_effect=match)
        inference_goals = []

        def waypoint_model(context, goal_image):
            selected = int(goal_image.item())
            inference_goals.append(selected)
            value = float("nan") if state.frame in invalid_frames else float(selected)
            return torch.zeros(1), torch.full((1, 8, 2), value)

        node = node if node is not None else SimpleNamespace()
        for key, value in dict(
            robot_config={"frame_rate": 5}, map_size=10, context_size=3,
            context_queue=[dict(timestamp=1., img=torch.zeros(1, 3, 2, 2),
                                vpr_img=torch.zeros(1, 3, 2, 2)) for _ in range(4)],
            obs_lock=Lock(), device="cpu", waypoint_scale=.2,
            topomap_images=[torch.tensor([float(i)]) for i in range(10)],
            place_recognition_querier=querier, waypoint_model=waypoint_model,
            waypoint_pub=Publisher(), viz_pub=Publisher(),
            localization_diagnostics_pub=Publisher(), goal_pub=Publisher(),
        ).items():
            setattr(node, key, value)

        def sleep():
            state.frame += 1

        ros = SimpleNamespace(
            is_shutdown=lambda: state.frame >= len(estimates),
            Rate=lambda hz: SimpleNamespace(sleep=sleep),
            loginfo=Mock(), loginfo_throttle=Mock(), logerr_throttle=Mock(),
        )
        namespace = dict(
            argparse=argparse, copy=copy, time=time, np=np, torch=torch, rospy=ros,
            to_numpy=lambda tensor: tensor.detach().cpu().numpy(),
            waypoints_to_ros_msg=lambda points: points.copy(),
            Viz=SimpleNamespace, LocalizationDiagnostics=SimpleNamespace,
            Float32MultiArray=SimpleNamespace,
        )
        tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "PlaceNavNode")
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "navigate")
        executable = ast.Module(body=[method], type_ignores=[])
        exec(compile(executable, str(SOURCE), "exec"), namespace)
        namespace["navigate"](node, args)
        return SimpleNamespace(node=node, inference_goals=inference_goals, ros=ros, querier=querier)

    def assert_published_goals(self, result, expected):
        self.assertEqual([m.subgoal_idx for m in result.node.viz_pub.messages], expected)
        self.assertEqual([m.subgoal_idx for m in result.node.localization_diagnostics_pub.messages], expected)
        for msg, goal in zip(result.node.viz_pub.messages, expected):
            np.testing.assert_allclose(msg.waypoints, goal * .2)

    def test_bayesian_jumps_keep_model_goal_and_outputs_to_at_most_one_step(self):
        result = self.run_navigation([2, 4, 1, 3, 6])
        expected = [2, 3, 3, 4, 5]
        self.assertEqual(result.inference_goals, expected)
        self.assert_published_goals(result, expected)
        self.assertEqual(
            [m.current_node_idx for m in result.node.localization_diagnostics_pub.messages],
            [2, 4, 1, 3, 6],
        )
        result.querier.initialize_model.assert_called_once()

    def test_sliding_window_uses_same_target_guard(self):
        result = self.run_navigation([2, 0, 3, 1], filter_mode="sliding_window", lookahead=2)
        self.assertEqual(result.inference_goals, [3, 3, 4, 4])
        self.assert_published_goals(result, [3, 3, 4, 4])
        result.querier.initialize_model.assert_not_called()

    def test_selecting_final_image_does_not_falsely_confirm_arrival(self):
        result = self.run_navigation([7, 3, 9, 0], lookahead=10)
        self.assertEqual(result.inference_goals, [9, 9, 9])
        self.assert_published_goals(result, [9, 9, 9])
        self.assertEqual(result.node.goal_pub.messages, [False, False, True])

    def test_new_navigation_run_resets_target_history(self):
        first = self.run_navigation([6])
        second = self.run_navigation([0], node=first.node)
        self.assertEqual(first.inference_goals, [2])
        self.assertEqual(second.inference_goals, [1])
        self.assert_published_goals(second, [1])

    def test_invalid_prediction_does_not_commit_forward_jump(self):
        result = self.run_navigation([2, 7, 1], invalid_frames=(1,))
        self.assertEqual(result.inference_goals, [2, 3, 2])
        self.assert_published_goals(result, [2, 2])
        self.assertEqual(
            [m.current_node_idx for m in result.node.localization_diagnostics_pub.messages],
            [2, 1],
        )
        result.ros.logerr_throttle.assert_called_once()

    def test_first_match_is_bounded_from_nonzero_start(self):
        result = self.run_navigation([9], start=4)
        self.assertEqual(result.inference_goals, [6])
        self.assert_published_goals(result, [6])
        self.assertEqual(result.node.goal_pub.messages, [False])
        behind = self.run_navigation([2], start=4)
        self.assertEqual(behind.inference_goals, [5])

    def test_far_final_node_estimate_does_not_bypass_target_limit(self):
        result = self.run_navigation([9, 9, 0])
        self.assertEqual(result.inference_goals, [2, 3, 3])
        self.assert_published_goals(result, [2, 3, 3])
        self.assertEqual(result.node.goal_pub.messages, [False, False, False])

    def test_end_of_map_caps_target_and_keeps_arrival_check(self):
        result = self.run_navigation([8, 8, 9], start=7, goal=-1)
        self.assertEqual(result.inference_goals, [9, 9, 9])
        self.assert_published_goals(result, [9, 9, 9])
        self.assertEqual(result.node.goal_pub.messages, [False, False, True])


if __name__ == "__main__":
    unittest.main()
