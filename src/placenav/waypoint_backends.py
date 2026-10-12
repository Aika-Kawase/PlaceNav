"""Goal-conditioned ViNT / NoMaD adapters. No downloads during navigation."""
from pathlib import Path
import math
import sys

import torch
from torch import nn

VISUALNAV_REV = "dca79815b704e5aa9c6bdc3082351f9e3b2848c2"
DIFFUSION_REV = "5ba07ac6661db573af695b419a7947ecb704690f"


def resolve_waypoint_scale(model_config, robot_config):
    """Resolve normalized XY scaling independently of the velocity limit."""
    value = model_config.get("metric_scale", 1.0)
    if value == "robot_step":
        max_v = float(robot_config["max_v"])
        frame_rate = float(robot_config["frame_rate"])
        if not all(math.isfinite(v) and v > 0 for v in (max_v, frame_rate)):
            raise ValueError("robot_step requires positive finite max_v and frame_rate")
        value = max_v / frame_rate
    if isinstance(value, bool):
        raise ValueError("metric_scale must be robot_step or a positive finite number")
    try:
        scale = float(value)
    except (TypeError, ValueError):
        raise ValueError("metric_scale must be robot_step or a positive finite number")
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("metric_scale must be robot_step or a positive finite number")
    return scale


def waypoint_transform(image_size):
    """Match upstream PIL resize (no center crop), followed by ImageNet norm."""
    from torchvision import transforms
    return transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize(tuple(reversed(image_size)), interpolation=transforms.InterpolationMode.BICUBIC),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])


def add_sources(weights_dir):
    root = Path(weights_dir) / "waypoint" / "sources"
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


def load_waypoint_backend(path, config, device):
    add_sources(Path(path).parent.parent)
    kind = config["model_type"]
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
    else:
        raise ValueError(f"Unsupported waypoint model: {kind}")
    # Only load trusted official checkpoints: legacy ViNT serializes an nn.Module.
    checkpoint = torch.load(path, map_location="cpu")
    state = checkpoint if kind == "nomad" else checkpoint["model"]
    if isinstance(state, nn.Module):
        state = getattr(state, "module", state).state_dict()
    model.load_state_dict(state, strict=True)
    if kind == "nomad":
        model = NoMaDAdapter(model, config)
    return model.to(device).eval()
