#!/usr/bin/env python3
"""Download pinned official inference sources and public pretrained checkpoints."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from placenav.waypoint_backends import VISUALNAV_REV, DIFFUSION_REV, OMNIVLA_REV

IDS = {"vint": "1ckrceGb5m_uUtq3pD8KHwnqtJgPl6kF5",
       "nomad": "1YJhkkMJAYOiKNyCaelbS_alpUpAJsOUb"}
# OmniVLA-edge is hosted on Hugging Face (MIT). Revision and hash are pinned.
OMNIVLA_HF = {"repo_id": "NHirose/omnivla-edge", "filename": "omnivla-edge.pth",
              "revision": "b1361b7e24f101edea795a98b00a826b61a97394",
              "sha256": "f93b056f124380899fce3e087a5c2b41ce4d8c07e211daad52b6ebf02e80d04d"}
MODELS = [*IDS, "omnivla_edge"]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def prepare_omnivla_edge(root):
    destination = root / "omnivla_edge.pth"
    if not destination.is_file():
        from huggingface_hub import hf_hub_download
        downloaded = Path(hf_hub_download(
            OMNIVLA_HF["repo_id"], OMNIVLA_HF["filename"], revision=OMNIVLA_HF["revision"]))
        shutil.copyfile(downloaded, destination.with_suffix(".pth.part"))
        destination.with_suffix(".pth.part").replace(destination)
    digest = sha256(destination)
    if digest != OMNIVLA_HF["sha256"]:
        raise RuntimeError(f"{destination}: sha256 {digest} != pinned {OMNIVLA_HF['sha256']}")
    destination.with_suffix(".json").write_text(json.dumps({
        "model": "omnivla_edge", "sha256": digest, "huggingface": OMNIVLA_HF,
        "omnivla_revision": OMNIVLA_REV,
    }, indent=2) + "\n")
    print(f"Prepared {destination}; run verify_waypoint.py before navigation.")


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
    parser.add_argument("--models", nargs="+", choices=MODELS, default=list(IDS))
    parser.add_argument("--weights-dir", type=Path, default=Path(__file__).parent / "model_weights")
    args = parser.parse_args()
    root = args.weights_dir.resolve() / "waypoint"
    (root / "sources").mkdir(parents=True, exist_ok=True)
    if set(args.models) & set(IDS):
        source(root / "sources", "visualnav-transformer",
               "https://github.com/robodhruv/visualnav-transformer.git", VISUALNAV_REV)
        source(root / "sources", "diffusion_policy",
               "https://github.com/real-stanford/diffusion_policy.git", DIFFUSION_REV)
    if "omnivla_edge" in args.models:
        source(root / "sources", "OmniVLA", "https://github.com/NHirose/OmniVLA.git", OMNIVLA_REV)
        prepare_omnivla_edge(root)
    for name in [m for m in args.models if m in IDS]:
        import gdown
        destination = root / f"{name}.pth"
        if not destination.is_file():
            temporary = destination.with_suffix(".pth.part")
            result = gdown.download(id=IDS[name], output=str(temporary), quiet=False)
            if result is None:
                raise RuntimeError(f"Download failed: {name}. Retry or copy official weights to {destination}")
            temporary.replace(destination)
        destination.with_suffix(".json").write_text(json.dumps({
            "model": name, "sha256": sha256(destination), "google_drive_id": IDS[name],
            "visualnav_revision": VISUALNAV_REV, "diffusion_revision": DIFFUSION_REV,
        }, indent=2) + "\n")
        print(f"Prepared {destination}; run verify_waypoint.py before navigation.")


if __name__ == "__main__":
    main()
