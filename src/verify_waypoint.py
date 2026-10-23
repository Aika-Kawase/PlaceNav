#!/usr/bin/env python3
"""Offline pretrained waypoint inference check. Does not publish ROS commands."""
import argparse
from pathlib import Path
import time

import numpy as np
from PIL import Image
import torch
import yaml

from placenav.waypoint_backends import load_waypoint_backend, waypoint_transform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", nargs="+", choices=["vint", "nomad"], default=["vint", "nomad"])
    parser.add_argument("--device", default="cuda", choices=["cpu", "cuda"])
    parser.add_argument("--weights-dir", type=Path, default=Path(__file__).parent / "model_weights")
    parser.add_argument("--image", type=Path, help="optional actual camera image; otherwise synthetic input")
    parser.add_argument("--nomad-candidates", type=int, default=1,
                        help="also validate N candidate generation against the single baseline")
    args = parser.parse_args()
    with (Path(__file__).parent / "config/models.yaml").open() as stream:
        configs = yaml.safe_load(stream)
    torch.set_num_threads(2)
    image = (Image.open(args.image).convert("RGB") if args.image else
             Image.fromarray(np.random.RandomState(0).randint(0, 256, (240, 320, 3), dtype=np.uint8)))
    for name in args.models:
        config = configs[name]
        model = load_waypoint_backend(args.weights_dir / config["checkpoint_path"], config, args.device)
        w, h = config["image_size"]
        goal = waypoint_transform([w, h])(np.asarray(image)).unsqueeze(0).to(args.device)
        obs = goal.repeat(1, config["context_size"] + 1, 1, 1)
        with torch.no_grad():
            distance, action = model(obs, goal)
            start = time.perf_counter()
            distance2, action2 = model(obs, goal)
            if args.device == "cuda":
                torch.cuda.synchronize()
            elapsed = time.perf_counter() - start
        assert action.shape == (1, config["len_traj_pred"], 4 if config["learn_angle"] else 2)
        assert torch.isfinite(action).all() and torch.isfinite(distance).all()
        assert torch.allclose(action, action2, atol=1e-5), "Repeated input differs"
        print(f"PASS {name}: input={w}x{h}, output={tuple(action.shape)}, inference={elapsed:.3f}s")
        if name == "nomad" and args.nomad_candidates > 1:
            with torch.no_grad():
                start = time.perf_counter()
                _, candidates = model.sample_candidates(obs, goal, args.nomad_candidates)
                if args.device == "cuda":
                    torch.cuda.synchronize()
                multi_elapsed = time.perf_counter()-start
                _, repeated = model.sample_candidates(obs, goal, args.nomad_candidates)
            assert candidates.shape == (1, args.nomad_candidates, config["len_traj_pred"], 2)
            assert torch.isfinite(candidates).all()
            assert torch.allclose(candidates[:, 0], action, atol=1e-5)
            assert torch.allclose(candidates, repeated, atol=1e-5)
            assert not torch.allclose(candidates[:, 0], candidates[:, 1])
            print(f"PASS nomad candidates={args.nomad_candidates}, inference={multi_elapsed:.3f}s; "
                  "candidate zero matches baseline; deterministic and distinct")
        del model
    print("Shape/finite/repeatability checks only; this does not establish navigation safety.")


if __name__ == "__main__":
    main()
