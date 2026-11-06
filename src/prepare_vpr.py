#!/usr/bin/env python3
"""Prepare self-contained, fixed-resolution VPR models before field deployment.

Run each backend in its own process. Upstream code is downloaded into the
ignored artifact directory, with its license and pinned Git revision intact.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess

os.environ['XFORMERS_DISABLED'] = '1'
import torch
from torch import nn
from torch.nn import functional as F

from placenav.place_recognition.vpr_config import sha256_file

REVISIONS = {
    'megaloc': ('gmberton/MegaLoc', '5fe0dd697c4a70ba3e23607f6716ab3c606b16db'),
    'salad': ('serizba/salad', '6aede13a3f6c25750bf7fde10209c06cb73060bb'),
    'dinov2': ('facebookresearch/dinov2', 'e1277af2ba9496fbadf7aec6eba56e8d882d1e35'),
}


def checkout(name, root):
    repo, revision = REVISIONS[name]
    dest = root / 'sources' / name
    git_env = dict(os.environ, GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM='1')
    if not dest.exists():
        dest.parent.mkdir(parents=True, exist_ok=True)
        url = 'https://github.com/' + repo + '.git'
        # A repository-specific rule overrides user-wide HTTPS -> SSH rewrites.
        subprocess.run(['git', '-c', f'url.{url}.insteadOf={url}', 'clone', url, str(dest)], check=True, env=git_env)
    subprocess.run(['git', '-C', str(dest), 'checkout', '--detach', revision], check=True)
    if subprocess.check_output(['git', '-C', str(dest), 'status', '--porcelain']).strip():
        raise RuntimeError(f'Upstream checkout contains changes: {dest}')
    return dest


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


class SaladInference(nn.Module):
    def __init__(self, backbone, aggregator):
        super().__init__()
        self.backbone = backbone
        self.aggregator = aggregator

    def forward(self, image):
        return self.aggregator(self.backbone(image))


class HardVLAD(nn.Module):
    """Cosine hard assignment + residual sum + intra/global normalization."""
    def __init__(self, centers):
        super().__init__()
        self.register_buffer('centers', centers.float())

    def forward(self, features):
        features = F.normalize(features, dim=-1)
        labels = (features @ F.normalize(self.centers, dim=-1).T).argmax(dim=-1)
        assignments = F.one_hot(labels, num_classes=self.centers.shape[0]).to(features.dtype)
        residuals = assignments.transpose(1, 2) @ features
        residuals = residuals - assignments.sum(dim=1).unsqueeze(-1) * self.centers
        return F.normalize(F.normalize(residuals, dim=-1).flatten(1), dim=-1)


class AnyLocInference(nn.Module):
    def __init__(self, backbone, centers):
        super().__init__()
        self.backbone = backbone
        self.vlad = HardVLAD(centers)
        self.values = None
        self.backbone.blocks[31].attn.qkv.register_forward_hook(self.capture)

    def capture(self, mod, inputs, output):
        self.values = output[:, 1:, output.shape[-1] * 2 // 3:]

    def forward(self, image):
        self.backbone(image)
        return self.vlad(self.values)


def build_model(args):
    if args.model == 'megaloc':
        from huggingface_hub import hf_hub_download
        from safetensors.torch import load_file
        src = checkout('megaloc', args.output_dir)
        model = module(src / 'megaloc_model.py', 'prepare_megaloc').MegaLoc()
        weights = hf_hub_download('gberton/MegaLoc', 'model.safetensors')
        model.load_state_dict(load_file(weights))
        return model, {'weights_sha256': sha256_file(weights)}

    dino_src = checkout('dinov2', args.output_dir)
    if args.model == 'salad':
        src = checkout('salad', args.output_dir)
        # Use official inference components without importing the training stack.
        backbone_cls = module(src / 'models/backbones/dinov2.py', 'prepare_salad_backbone').DINOv2
        original_load = torch.hub.load
        def local_dino(repo, name, *pos, **kw):
            if repo == 'facebookresearch/dinov2':
                return original_load(str(dino_src), name, source='local', pretrained=False)
            return original_load(repo, name, *pos, **kw)
        torch.hub.load = local_dino
        try:
            backbone = backbone_cls('dinov2_vitb14', num_trainable_blocks=4, norm_layer=True, return_token=True)
        finally:
            torch.hub.load = original_load
        aggregator_cls = module(src / 'models/aggregators/salad.py', 'prepare_salad_aggregator').SALAD
        model = SaladInference(backbone, aggregator_cls(num_channels=768))
        weights = torch.hub.load_state_dict_from_url(
            'https://github.com/serizba/salad/releases/download/v1.0.0/dino_salad.ckpt', map_location='cpu')
        model.load_state_dict(weights)
        return model, {'weights_url': 'https://github.com/serizba/salad/releases/download/v1.0.0/dino_salad.ckpt'}

    if args.centers is None:
        args.centers = args.output_dir / 'dinov2_vitg14_l31_value_c32_urban_c_centers.pt'
        if not args.centers.exists():
            torch.hub.download_url_to_file(
                'https://github.com/AnyLoc/DINO/releases/download/v1/' + args.centers.name,
                str(args.centers))
    if not args.centers.is_file():
        raise FileNotFoundError(args.centers)
    centers = torch.load(args.centers, map_location='cpu')
    if centers.shape != (32, 1536) or not torch.isfinite(centers).all():
        raise ValueError('Expected finite AnyLoc urban vocabulary of shape [32, 1536]')
    backbone = torch.hub.load(str(dino_src), 'dinov2_vitg14', source='local', pretrained=True)
    return AnyLocInference(backbone, centers), {
        'centers_sha256': sha256_file(args.centers), 'layer': 31, 'facet': 'value', 'clusters': 32,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True, choices=['megaloc', 'salad', 'anyloc'])
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent / 'model_weights/vpr')
    parser.add_argument('--centers', type=Path, help='Official AnyLoc urban c_centers.pt')
    parser.add_argument('--device', default='cpu', choices=['cpu', 'cuda'])
    parser.add_argument('--force', action='store_true', help='Replace an existing export')
    args = parser.parse_args()
    if not hasattr(F, 'scaled_dot_product_attention'):
        raise RuntimeError('Use the updated container (PyTorch 2.1.2), then run this command again')
    args.output_dir = args.output_dir.resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    dest = args.output_dir / (args.model + '.pt')
    if dest.exists() and not args.force:
        raise FileExistsError(f'{dest} exists; use --force to replace it')
    model, provenance = build_model(args)
    model = model.eval().to(args.device)
    torch.manual_seed(0)
    sample = torch.randn(1, 3, 322, 322, device=args.device)
    dimensions = 49152 if args.model == 'anyloc' else 8448
    # Trace at deployment shape; retain parameters as tensors so .to(cuda) works.
    with torch.inference_mode():
        traced = torch.jit.trace(model, sample, check_trace=False)
        for probe in (sample, torch.randn_like(sample)):
            expected, actual = model(probe), traced(probe)
            torch.testing.assert_close(actual, expected, rtol=1e-4, atol=1e-5)
            if actual.shape != (1, dimensions) or not torch.isfinite(actual).all():
                raise RuntimeError('Export produced invalid descriptors')
            torch.testing.assert_close(actual.norm(dim=-1), torch.ones(1, device=args.device))
    metadata = dict(backend=args.model, input_size=[322, 322], descriptor_dim=dimensions,
                    torch_version=torch.__version__, revisions=REVISIONS, **provenance)
    temp = dest.with_suffix('.building')
    traced.save(str(temp), _extra_files={'metadata.json': json.dumps(metadata)})
    # Verify loading with no upstream Python code required.
    loaded = torch.jit.load(str(temp), map_location=args.device).eval()
    with torch.inference_mode():
        torch.testing.assert_close(loaded(sample), model(sample), rtol=1e-4, atol=1e-5)
    temp.replace(dest)
    print(f'Ready: {dest} ({dimensions} dimensions). Copy this file to the robot at the same relative path.')


if __name__ == '__main__':
    main()
