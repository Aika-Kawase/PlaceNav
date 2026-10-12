import sys
import unittest
from pathlib import Path

import torch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from placenav.waypoint_backends import decode_actions, NoMaDAdapter, resolve_waypoint_scale


class FakeNoMaD(torch.nn.Module):
    def forward(self, operation, **kwargs):
        if operation == "vision_encoder":
            assert not kwargs["input_goal_mask"].any()
            return torch.zeros(kwargs["obs_img"].shape[0], 256)
        if operation == "dist_pred_net":
            return torch.ones(kwargs["obsgoal_cond"].shape[0], 1)
        return torch.zeros_like(kwargs["sample"])


class WaypointTests(unittest.TestCase):
    def test_metric_scale_modes(self):
        robot = {"max_v": 0.4, "frame_rate": 5}
        self.assertEqual(resolve_waypoint_scale({}, robot), 1.0)
        self.assertAlmostEqual(resolve_waypoint_scale({"metric_scale": "robot_step"}, robot), 0.08)
        for robot_config in (robot, {"max_v": 0.2, "frame_rate": 10}):
            self.assertEqual(resolve_waypoint_scale({"metric_scale": 0.16}, robot_config), 0.16)

    def test_invalid_metric_scales(self):
        for value in (0, -1, float("nan"), float("inf"), None, True, "typo"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                resolve_waypoint_scale({"metric_scale": value}, {})
        with self.assertRaises(ValueError):
            resolve_waypoint_scale({"metric_scale": "robot_step"}, {"max_v": 0.4, "frame_rate": 0})

    def test_decode_bounds_and_cumulative_sum(self):
        sample = torch.tensor([[[-1., -1.], [1., 1.], [0., 0.]]])
        torch.testing.assert_close(decode_actions(sample),
                                   torch.tensor([[[-2.5, -4.], [2.5, 0.], [3.75, 0.]]]))

    def test_nomad_goal_conditioned_reproducible(self):
        model = NoMaDAdapter(FakeNoMaD(), {"len_traj_pred": 8, "num_diffusion_iters": 10})
        obs, goal = torch.zeros(1, 12, 96, 96), torch.zeros(1, 3, 96, 96)
        dist, action = model(obs, goal)
        self.assertEqual(action.shape, (1, 8, 2))
        torch.testing.assert_close(action, model(obs, goal)[1])
        self.assertTrue(torch.isfinite(action).all())

    def test_model_config(self):
        with (Path(__file__).resolve().parents[1] / "src/config/models.yaml").open() as f:
            conf = yaml.safe_load(f)
        self.assertEqual(conf["gnm_large"]["image_size"], [85, 64])
        self.assertEqual(conf["vint"]["len_traj_pred"], 5)
        self.assertEqual(conf["nomad"]["len_traj_pred"], 8)
        self.assertEqual(conf["nomad"]["image_size"], [96, 96])


if __name__ == "__main__":
    unittest.main()
