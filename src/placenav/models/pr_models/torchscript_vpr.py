"""Offline inference for models exported by prepare_vpr.py."""
import json
from pathlib import Path

import torch
import torch.nn.functional as F

from .base_model import BaseModel


class TorchScriptVPR(BaseModel):
    required_inputs = ['image']

    def _init(self, conf):
        path = Path(conf['checkpoint_path'])
        if not path.is_file():
            raise FileNotFoundError(
                f'{path} is missing. Run python3 prepare_vpr.py '
                f'--model {conf["backend"]} on an Internet-connected machine first.')
        extra = {'metadata.json': ''}
        self.net = torch.jit.load(str(path), map_location='cpu', _extra_files=extra).eval()
        metadata = json.loads(extra['metadata.json'])
        for key in ('backend', 'input_size', 'descriptor_dim'):
            if metadata[key] != conf[key]:
                raise ValueError(f'Export/config mismatch for {key}: {metadata[key]} != {conf[key]}')
        self._graph_device = None

    def _forward(self, data):
        image = data['image']
        if self._graph_device != image.device:
            # Tracing can embed a device in allocations (e.g. SALAD's Sinkhorn).
            # Module.to() moves weights but not these device literals.
            from .script_device import set_graph_device
            self.net = set_graph_device(self.net, image.device)
            self._graph_device = image.device
        if list(image.shape[-2:][::-1]) != self.conf['input_size']:
            raise ValueError('VPR image size does not match exported model')
        desc = F.normalize(self.net(image).float(), dim=-1)
        if desc.ndim != 2 or desc.shape[1] != self.conf['descriptor_dim']:
            raise ValueError('Unexpected VPR descriptor dimension')
        if not torch.isfinite(desc).all() or (desc.norm(dim=-1) < 0.99).any():
            raise ValueError('Invalid VPR descriptor')
        return {'global_descriptor': desc}
