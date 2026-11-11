import sys
import unittest
from pathlib import Path

import torch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from placenav.waypoint_backends import (
    add_sources, decode_actions, load_waypoint_backend, NoMaDAdapter, OmniVLAEdgeAdapter)

ROOT = Path(__file__).resolve().parents[1]
OMNIVLA_SOURCES = ROOT / "src/model_weights/waypoint/sources/OmniVLA/inference"
HAS_OMNIVLA = (OMNIVLA_SOURCES / "model_omnivla_edge.py").is_file()


class FakeNoMaD(torch.nn.Module):
    def forward(self, operation, **kwargs):
        if operation == "vision_encoder":
            assert not kwargs["input_goal_mask"].any()
            return torch.zeros(kwargs["obs_img"].shape[0], 256)
        if operation == "dist_pred_net":
            return torch.ones(kwargs["obsgoal_cond"].shape[0], 1)
        return torch.zeros_like(kwargs["sample"])


class WaypointTests(unittest.TestCase):
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

    def test_omnivla_edge_config(self):
        with (ROOT / "src/config/models.yaml").open() as f:
            conf = yaml.safe_load(f)["omnivla_edge"]
        self.assertEqual(conf["image_size"], [96, 96])
        self.assertEqual((conf["context_size"], conf["len_traj_pred"]), (5, 8))
        self.assertEqual(conf["metric_scale"], 0.1)  # numeric: fixed metres per unit
        self.assertTrue(conf["learn_angle"])


WEIGHTS = ROOT / "src/model_weights/waypoint/omnivla_edge.pth"


class OmniVLAEdgeCpuTests(unittest.TestCase):
    @unittest.skipUnless(HAS_OMNIVLA, "run src/prepare_waypoint.py --models omnivla_edge first")
    def test_cpu_is_rejected_with_a_clear_error(self):
        with (ROOT / "src/config/models.yaml").open() as f:
            conf = yaml.safe_load(f)["omnivla_edge"]
        with self.assertRaisesRegex(RuntimeError, "CUDA"):
            load_waypoint_backend(WEIGHTS, conf, "cpu")


# Real weights are required: a randomly initialised network ends in ReLU and
# outputs a constant, which would make every sensitivity check below vacuous.
@unittest.skipUnless(HAS_OMNIVLA and WEIGHTS.is_file() and torch.cuda.is_available(),
                     "needs CUDA and src/prepare_waypoint.py --models omnivla_edge")
class OmniVLAEdgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with (ROOT / "src/config/models.yaml").open() as f:
            conf = yaml.safe_load(f)["omnivla_edge"]
        cls.adapter = load_waypoint_backend(WEIGHTS, conf, "cuda")
        torch.manual_seed(0)
        cls.obs = torch.randn(2, 18, 96, 96).cuda()
        cls.goal = torch.randn(2, 3, 96, 96).cuda()

    def raw(self, modality, pose, text):
        ids = torch.full((2,), modality, dtype=torch.long, device="cuda")
        with torch.no_grad():
            action, dist, _ = self.adapter.model(
                self.obs, pose, torch.zeros(2, 9, 96, 96).cuda(), self.goal, ids, text,
                torch.zeros(2, 3, 224, 224).cuda())
        return action

    def test_shape_and_unit_heading(self):
        with torch.no_grad():
            dist, action = self.adapter(self.obs, self.goal)
        self.assertEqual(action.shape, (2, 8, 4))
        self.assertEqual(dist.shape, (2, 1))
        self.assertTrue(torch.isfinite(action).all() and torch.isfinite(dist).all())
        torch.testing.assert_close(action[..., 2:].norm(dim=-1), torch.ones(2, 8, device="cuda"))

    def test_unused_modalities_cannot_influence_goal_image_mode(self):
        """Zeros for pose/map/language are exact stand-ins: the network masks them."""
        with torch.no_grad():
            _, action = self.adapter(self.obs, self.goal)
        noisy = self.raw(6, torch.randn(2, 4).cuda(), torch.randn(2, 512).cuda())
        torch.testing.assert_close(action, noisy)
        # Negative control: a mode that does read the goal pose must react to it.
        zero = self.raw(4, torch.zeros(2, 4).cuda(), torch.zeros(2, 512).cuda())
        moved = self.raw(4, torch.randn(2, 4).cuda() * 20, torch.zeros(2, 512).cuda())
        self.assertFalse(torch.allclose(zero, moved), "control failed: pose mode ignores the pose")

    def test_goal_image_matters_and_batching_is_consistent(self):
        with torch.no_grad():
            _, both = self.adapter(self.obs, self.goal)
            _, first = self.adapter(self.obs[:1], self.goal[:1])
            _, swapped = self.adapter(self.obs[:1], self.goal[1:])
            _, again = self.adapter(self.obs[:1], self.goal[:1])
        torch.testing.assert_close(both[:1], first, atol=1e-4, rtol=1e-4)
        torch.testing.assert_close(first, again)  # deterministic, unlike NoMaD sampling
        self.assertFalse(torch.allclose(first, swapped), "goal image has no effect")


if __name__ == "__main__":
    unittest.main()
