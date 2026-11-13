#!/usr/bin/env python3
"""Measure VPR and waypoint inference latency on this device. No ROS, no topomap.

Inputs are random tensors at the deployment resolution, so the numbers show
compute cost only (not accuracy). Run it on the Xavier before deciding which
models it should host, e.g.

    python3 bench_inference.py --pr-models cosplace salad --wp-models omnivla_edge nomad
"""
import argparse
from pathlib import Path
import sys
import time

import torch
import yaml

SRC = Path(__file__).resolve().parent
sys.path.insert(0, str(SRC / "placenav"))  # navigate.py's own imports (utils, ...)

from placenav.place_recognition.feature_extractor import FeatureExtractor  # noqa: E402
from utils import load_model  # noqa: E402
PR_MODELS = ["cosplace", "megaloc", "salad", "anyloc"]
WP_MODELS = ["gnm_large", "vint", "nomad", "omnivla_edge"]


def measure(step, iterations, warmup, cuda):
    with torch.no_grad():
        for _ in range(warmup):
            step()
        if cuda:
            torch.cuda.synchronize()
        times = []
        for _ in range(iterations):
            start = time.perf_counter()
            step()
            if cuda:
                torch.cuda.synchronize()
            times.append((time.perf_counter() - start) * 1e3)
    times.sort()
    return times[len(times) // 2], times[max(0, int(round(0.9 * len(times))) - 1)], times[-1]


def vpr_step(name, configs, weights_dir, device):
    conf = configs[name]
    conf["model"]["checkpoint_path"] = weights_dir / conf["model"]["checkpoint_path"]
    # Same default as navigate.py: CosPlace keeps its own resolution.
    width, height = conf.get("preprocessing", {"resize": [85, 64]})["resize"]
    extractor = FeatureExtractor(conf, device)
    image = torch.randn(1, 3, height, width, device=device)
    return lambda: extractor(image), f"{width}x{height}"


def waypoint_step(name, configs, weights_dir, device):
    config = configs[name]
    config["seed"] = 0
    width, height = config["image_size"]
    config["img_size"] = (width, height)
    model = load_model(weights_dir / config["checkpoint_path"], config, device).eval()
    context = torch.randn(1, 3 * (config["context_size"] + 1), height, width, device=device)
    goal = torch.randn(1, 3, height, width, device=device)
    return lambda: model(context, goal), f"{width}x{height}x{config['context_size'] + 1}"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pr-models", nargs="*", choices=PR_MODELS, default=["cosplace"])
    parser.add_argument("--wp-models", nargs="*", choices=WP_MODELS, default=["omnivla_edge"])
    parser.add_argument("--device", default="cuda", choices=["cpu", "cuda"])
    parser.add_argument("--iterations", type=int, default=50)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--weights-dir", type=Path, default=SRC / "model_weights")
    parser.add_argument("--model-config", type=Path, default=SRC / "config/models.yaml")
    args = parser.parse_args()
    device = torch.device(args.device)
    cuda = device.type == "cuda"
    with args.model_config.open() as stream:
        configs = yaml.safe_load(stream)
    print(f"torch {torch.__version__}, device {torch.cuda.get_device_name() if cuda else 'cpu'}")
    print(f"{'kind':<9}{'model':<14}{'input':<14}{'median ms':>10}{'p90 ms':>9}{'max ms':>9}")
    jobs = [("vpr", name, vpr_step) for name in args.pr_models]
    jobs += [("waypoint", name, waypoint_step) for name in args.wp_models]
    failed = False
    for kind, name, build in jobs:
        try:
            step, shape = build(name, configs, args.weights_dir, device)
            median, p90, worst = measure(step, args.iterations, args.warmup, cuda)
        except Exception as error:  # report every model even if one is missing
            print(f"{kind:<9}{name:<14}FAILED: {type(error).__name__}: {error}")
            failed = True
            continue
        print(f"{kind:<9}{name:<14}{shape:<14}{median:>10.1f}{p90:>9.1f}{worst:>9.1f}")
        if cuda:
            torch.cuda.empty_cache()
    if cuda:
        print(f"peak GPU memory: {torch.cuda.max_memory_allocated() / 2**20:.0f} MiB")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
