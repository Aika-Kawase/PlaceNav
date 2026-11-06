#!/usr/bin/env python3
"""Download pinned official inference sources and public pretrained checkpoints."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from placenav.waypoint_backends import VISUALNAV_REV, DIFFUSION_REV

IDS = {"vint": "1ckrceGb5m_uUtq3pD8KHwnqtJgPl6kF5",
       "nomad": "1YJhkkMJAYOiKNyCaelbS_alpUpAJsOUb"}


def source(root, name, url, revision):
    path = root / name
    if not path.exists():
        # Override global HTTPS->SSH rewrites for these public repositories.
        subprocess.run(["git", "-c", f"url.{url}.insteadOf={url}",
                        "clone", "--no-checkout", url, str(path)], check=True)
        subprocess.run(["git", "-C", str(path), "checkout", "--detach", revision], check=True)
    actual = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
    if actual != revision:
        raise RuntimeError(f"{path}: expected {revision}, got {actual}; use a fresh weights directory")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", nargs="+", choices=IDS, default=list(IDS))
    parser.add_argument("--weights-dir", type=Path, default=Path(__file__).parent / "model_weights")
    args = parser.parse_args()
    root = args.weights_dir.resolve() / "waypoint"
    (root / "sources").mkdir(parents=True, exist_ok=True)
    source(root / "sources", "visualnav-transformer",
           "https://github.com/robodhruv/visualnav-transformer.git", VISUALNAV_REV)
    source(root / "sources", "diffusion_policy",
           "https://github.com/real-stanford/diffusion_policy.git", DIFFUSION_REV)
    import gdown
    for name in args.models:
        destination = root / f"{name}.pth"
        if not destination.is_file():
            temporary = destination.with_suffix(".pth.part")
            result = gdown.download(id=IDS[name], output=str(temporary), quiet=False)
            if result is None:
                raise RuntimeError(f"Download failed: {name}. Retry or copy official weights to {destination}")
            temporary.replace(destination)
        digest = hashlib.sha256()
        with destination.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        destination.with_suffix(".json").write_text(json.dumps({
            "model": name, "sha256": digest.hexdigest(), "google_drive_id": IDS[name],
            "visualnav_revision": VISUALNAV_REV, "diffusion_revision": DIFFUSION_REV,
        }, indent=2) + "\n")
        print(f"Prepared {destination}; run verify_waypoint.py before navigation.")


if __name__ == "__main__":
    main()
