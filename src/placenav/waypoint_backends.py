"""Goal-conditioned ViNT / NoMaD / OmniVLA-edge adapters. No downloads during navigation."""
from pathlib import Path
import sys

import torch
from torch import nn

VISUALNAV_REV = "dca79815b704e5aa9c6bdc3082351f9e3b2848c2"
DIFFUSION_REV = "5ba07ac6661db573af695b419a7947ecb704690f"
OMNIVLA_REV = "5182600cb4a9ee07684e17cdd2a6cbafc56b8a68"


def waypoint_transform(image_size):
    """Match upstream PIL resize (no center crop), followed by ImageNet norm."""
    from torchvision import transforms
    return transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize(tuple(reversed(image_size)), interpolation=transforms.InterpolationMode.BICUBIC),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])


def add_sources(weights_dir, kind="vint"):
    root = Path(weights_dir) / "waypoint" / "sources"
    if kind == "omnivla_edge":
        # Only the self-contained architecture file is needed. The upstream
        # inference script itself (CLIP, matplotlib, utm, ...) is never imported.
        path = root / "OmniVLA" / "inference"
        if not (path / "model_omnivla_edge.py").is_file():
            raise FileNotFoundError(f"Missing {path}. Run src/prepare_waypoint.py --models omnivla_edge first.")
        sys.path.insert(0, str(path))
        return
    for name, suffix in (("visualnav-transformer", "train"), ("diffusion_policy", "")):
        path = root / name / suffix
        if not path.is_dir():
            raise FileNotFoundError(f"Missing {path}. Run src/prepare_waypoint.py first.")
        sys.path.insert(0, str(path))


def decode_actions(sample):
    """Official NoMaD delta unnormalization, followed by cumulative sum."""
    low = sample.new_tensor([-2.5, -4.0])
    high = sample.new_tensor([5.0, 4.0])
    return ((sample + 1) * 0.5 * (high - low) + low).cumsum(dim=1)


class NoMaDAdapter(nn.Module):
    def __init__(self, model, config):
        super().__init__()
        from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
        self.model = model
        self.horizon = config["len_traj_pred"]
        self.steps = config["num_diffusion_iters"]
        self.seed = config.get("seed", 0)
        self.scheduler = DDPMScheduler(
            num_train_timesteps=self.steps, beta_schedule="squaredcos_cap_v2",
            clip_sample=True, prediction_type="epsilon")

    def forward(self, observation, goal):
        # Goal mask=0: follow the VPR-selected image, NOT goal-free exploration.
        cond = self.model("vision_encoder", obs_img=observation, goal_img=goal,
                          input_goal_mask=torch.zeros(observation.shape[0],
                                                      device=observation.device, dtype=torch.long))
        distance = self.model("dist_pred_net", obsgoal_cond=cond)
        # Fixed per-call seed makes repeated inputs reproducible. One trajectory,
        # never an average of incompatible left/right avoidance hypotheses.
        generator = torch.Generator(device=observation.device).manual_seed(self.seed)
        action = torch.randn((observation.shape[0], self.horizon, 2),
                             device=observation.device, generator=generator)
        self.scheduler.set_timesteps(self.steps)
        for timestep in self.scheduler.timesteps:
            noise = self.model("noise_pred_net", sample=action, timestep=timestep,
                               global_cond=cond)
            action = self.scheduler.step(noise, timestep, action, generator=generator).prev_sample
        return distance, decode_actions(action)


class OmniVLAEdgeAdapter(nn.Module):
    """OmniVLA-edge conditioned on the goal image only (upstream modality id 6).

    The unused pose / satellite-map / language inputs are masked inside the
    network, so neutral zeros are exact stand-ins. This keeps the goal-image
    interface shared with ViNT/NoMaD and needs neither CLIP nor GPS.
    Actions are x-forward / y-left, [dx, dy, cos, sin], in units of 0.1 m.
    """
    IMAGE_ONLY = 6
    TEXT_DIM = 512
    LARGE_IMAGE_SIZE = 224

    def __init__(self, model, config):
        super().__init__()
        self.model = model
        self.context_size = config["context_size"]
        self.horizon = config["len_traj_pred"]

    def forward(self, observation, goal):
        batch, device = observation.shape[0], observation.device
        size = observation.shape[-1]
        zeros = lambda *shape: torch.zeros(batch, *shape, device=device, dtype=observation.dtype)
        modality = torch.full((batch,), self.IMAGE_ONLY, device=device, dtype=torch.long)
        action, distance, _ = self.model(
            observation, zeros(4), zeros(9, size, size), goal, modality,
            zeros(self.TEXT_DIM), zeros(3, self.LARGE_IMAGE_SIZE, self.LARGE_IMAGE_SIZE))
        return distance, action


def load_waypoint_backend(path, config, device):
    kind = config["model_type"]
    add_sources(Path(path).parent.parent, kind)
    if kind == "vint":
        from vint_train.models.vint.vint import ViNT
        names = ("context_size", "len_traj_pred", "learn_angle", "obs_encoder",
                 "obs_encoding_size", "late_fusion", "mha_num_attention_heads",
                 "mha_num_attention_layers", "mha_ff_dim_factor")
        model = ViNT(**{key: config[key] for key in names})
    elif kind == "nomad":
        from vint_train.models.nomad.nomad import NoMaD, DenseNetwork
        from vint_train.models.nomad.nomad_vint import NoMaD_ViNT, replace_bn_with_gn
        from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D
        encoder = NoMaD_ViNT(
            obs_encoding_size=config["encoding_size"], context_size=config["context_size"],
            mha_num_attention_heads=config["mha_num_attention_heads"],
            mha_num_attention_layers=config["mha_num_attention_layers"],
            mha_ff_dim_factor=config["mha_ff_dim_factor"])
        model = NoMaD(
            replace_bn_with_gn(encoder),
            ConditionalUnet1D(input_dim=2, global_cond_dim=config["encoding_size"],
                              down_dims=config["down_dims"],
                              cond_predict_scale=config["cond_predict_scale"]),
            DenseNetwork(config["encoding_size"]))
    elif kind == "omnivla_edge":
        if torch.device(device).type != "cuda":
            # Upstream calls Tensor.get_device() to place its masks, which is -1 on CPU.
            raise RuntimeError("OmniVLA-edge needs --device cuda (upstream model code is CUDA-only)")
        from model_omnivla_edge import OmniVLA_edge
        names = ("context_size", "len_traj_pred", "learn_angle", "obs_encoder",
                 "obs_encoding_size", "late_fusion", "mha_num_attention_heads",
                 "mha_num_attention_layers", "mha_ff_dim_factor")
        model = OmniVLA_edge(**{key: config[key] for key in names})
    else:
        raise ValueError(f"Unsupported waypoint model: {kind}")
    # Only load trusted official checkpoints: legacy ViNT serializes an nn.Module.
    # OmniVLA-edge ships a plain state dict, so it never needs unpickling.
    checkpoint = torch.load(path, map_location="cpu", weights_only=(kind == "omnivla_edge"))
    state = checkpoint if kind in ("nomad", "omnivla_edge") else checkpoint["model"]
    if isinstance(state, nn.Module):
        state = getattr(state, "module", state).state_dict()
    model.load_state_dict(state, strict=True)
    if kind == "nomad":
        model = NoMaDAdapter(model, config)
    elif kind == "omnivla_edge":
        model = OmniVLAEdgeAdapter(model, config)
    return model.to(device).eval()
