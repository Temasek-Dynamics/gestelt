import torch
import torch.nn as nn
import torch.nn.functional as F


# =========================================================
# Utilities: Normalization
# =========================================================

class RunningNorm(nn.Module):
    """
    Online running mean/std normalizer for vector inputs.
    - Use update=True during training to update running stats.
    - Use update=False for evaluation.
    """
    def __init__(self, dim: int, eps: float = 1e-6, momentum: float = 0.01, clip: float = 5.0):
        super().__init__()
        self.dim = dim
        self.eps = eps
        self.momentum = momentum
        self.clip = clip

        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("var", torch.ones(dim))
        self.register_buffer("initialized", torch.tensor(False))

    @torch.no_grad()
    def _init_from_batch(self, x: torch.Tensor):
        m = x.mean(dim=0)
        v = x.var(dim=0, unbiased=False).clamp_min(self.eps)
        self.mean.copy_(m)
        self.var.copy_(v)
        self.initialized.fill_(True)

    @torch.no_grad()
    def _update(self, x: torch.Tensor):
        if not bool(self.initialized.item()):
            self._init_from_batch(x)
            return
        batch_mean = x.mean(dim=0)
        batch_var = x.var(dim=0, unbiased=False).clamp_min(self.eps)

        self.mean.lerp_(batch_mean, self.momentum)
        self.var.lerp_(batch_var, self.momentum)

    def forward(self, x: torch.Tensor, update: bool = False) -> torch.Tensor:
        # x: [B, dim]
        if update:
            self._update(x)

        x_hat = (x - self.mean) / torch.sqrt(self.var + self.eps)
        if self.clip is not None:
            x_hat = torch.clamp(x_hat, -self.clip, self.clip)
        return x_hat


class ImageNormalizer(nn.Module):
    """
    Normalize image tensors.
    Supports:
      - [B,H,W] -> becomes [B,1,H,W]
      - [B,1,H,W] or [B,C,H,W]
    Pixel range:
      - if max>1.5 -> assumed 0..255 -> /255
      - else assumed already 0..1 (e.g., binary 0/1)
    Optional per-channel mean/std standardization.
    """
    def __init__(self, mean=None, std=None, eps: float = 1e-6):
        super().__init__()
        self.eps = eps

        if mean is None:
            self.register_buffer("mean", None)
        else:
            mean = torch.tensor(mean, dtype=torch.float32).view(1, -1, 1, 1)
            self.register_buffer("mean", mean)

        if std is None:
            self.register_buffer("std", None)
        else:
            std = torch.tensor(std, dtype=torch.float32).view(1, -1, 1, 1)
            self.register_buffer("std", std)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.dim() == 3:     # [B,H,W]
            x = x.unsqueeze(1)
        x = x.float()

        # auto-scale if looks like 0..255
        if x.max() > 1.5:
            x = x / 255.0

        if (self.mean is not None) and (self.std is not None):
            x = (x - self.mean) / (self.std + self.eps)

        return x


# =========================================================
# 1) TrackVel (unchanged output constraint, optional state norm)
# =========================================================

class TrackVel(nn.Module):
    """
    Track velocity MLP
    Input dim (13) = attitude (4) + qd (6) + target velocity (3)
    Output dim (4): a0 in (0,1), a1..a3 in (-1,1)
    """
    def __init__(self, input_dim: int = 13, output_dim: int = 4, use_state_norm: bool = True):
        super().__init__()
        self.use_state_norm = use_state_norm
        self.state_norm = RunningNorm(input_dim) if use_state_norm else None

        self.fc1 = nn.Linear(input_dim, 64)
        self.fc2 = nn.Linear(64, 128)
        self.fc3 = nn.Linear(128, 256)
        self.fc4 = nn.Linear(256, 512)
        self.fc5 = nn.Linear(512, 256)
        self.fc6 = nn.Linear(256, 128)
        self.fc7 = nn.Linear(128, 64)
        self.fc8 = nn.Linear(64, 32)
        self.output_layer = nn.Linear(32, output_dim)

    def forward(self, x: torch.Tensor, update_norm: bool = False) -> torch.Tensor:
        # x: [B,input_dim]
        if self.use_state_norm:
            x = self.state_norm(x, update=update_norm)

        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        x = F.relu(self.fc5(x))
        x = F.relu(self.fc6(x))
        x = F.relu(self.fc7(x))
        x = F.relu(self.fc8(x))

        out = self.output_layer(x)
        first = torch.sigmoid(out[:, 0:1])   # (0,1)
        rest = torch.tanh(out[:, 1:])        # (-1,1)
        return torch.cat((first, rest), dim=1)


# =========================================================
# 2) CNN / Vision Encoder (improved + generic)
# =========================================================

class CNNImageEncoder(nn.Module):
    """
    Generic CNN encoder -> latent vector.
    Default:
      in_channels=1 (binary or depth/gray)
      latent_dims=64
    """
    def __init__(self, image_res=(128, 128), in_channels: int = 1, latent_dims: int = 64):
        super().__init__()
        self.image_res = image_res
        self.latent_dims = latent_dims
        self.in_channels = in_channels

        self.features = nn.Sequential(
            nn.Conv2d(in_channels, 32, kernel_size=5, stride=2, padding=2),
            nn.ELU(),
            nn.Conv2d(32, 64, kernel_size=5, stride=2, padding=2),
            nn.ELU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.ELU(),
            nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1),
            nn.ELU(),
        )

        self.projection = nn.Sequential(
            nn.AdaptiveAvgPool2d(1),
            nn.Conv2d(256, latent_dims, kernel_size=1),
            nn.Flatten(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: [B,H,W] or [B,C,H,W]
        if x.dim() == 3:
            x = x.unsqueeze(1)
        feats = self.features(x)
        latent = self.projection(feats)
        return latent  # [B,latent_dims]


class ChannelAttention(nn.Module):
    def __init__(self, channels: int, reduction: int = 16):
        super().__init__()
        hidden = max(1, channels // reduction)
        self.avg_pool = nn.AdaptiveAvgPool2d(1)
        self.max_pool = nn.AdaptiveMaxPool2d(1)
        self.mlp = nn.Sequential(
            nn.Conv2d(channels, hidden, kernel_size=1, bias=False),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden, channels, kernel_size=1, bias=False),
        )
        self.sigmoid = nn.Sigmoid()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        attn = self.mlp(self.avg_pool(x)) + self.mlp(self.max_pool(x))
        return self.sigmoid(attn)


class SpatialAttention(nn.Module):
    def __init__(self, kernel_size: int = 7):
        super().__init__()
        if kernel_size not in (3, 7):
            raise ValueError("SpatialAttention kernel_size must be 3 or 7.")
        padding = 3 if kernel_size == 7 else 1
        self.conv = nn.Conv2d(2, 1, kernel_size=kernel_size, padding=padding, bias=False)
        self.sigmoid = nn.Sigmoid()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        avg_map = torch.mean(x, dim=1, keepdim=True)
        max_map, _ = torch.max(x, dim=1, keepdim=True)
        attn = torch.cat([avg_map, max_map], dim=1)
        return self.sigmoid(self.conv(attn))


class CBAMBlock(nn.Module):
    def __init__(self, channels: int, reduction: int = 16, spatial_kernel_size: int = 7):
        super().__init__()
        self.channel_attn = ChannelAttention(channels=channels, reduction=reduction)
        self.spatial_attn = SpatialAttention(kernel_size=spatial_kernel_size)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x * self.channel_attn(x)
        x = x * self.spatial_attn(x)
        return x


class CNNImageEncoderCBAMLastTwo(nn.Module):
    """
    CNN encoder with CBAM inserted after the last two conv blocks.
    """
    def __init__(self, image_res=(128, 128), in_channels: int = 1, latent_dims: int = 64):
        super().__init__()
        self.image_res = image_res
        self.latent_dims = latent_dims
        self.in_channels = in_channels

        self.conv1 = nn.Conv2d(in_channels, 32, kernel_size=5, stride=2, padding=2)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=5, stride=2, padding=2)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1)
        self.conv4 = nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1)
        self.act = nn.ELU()

        self.cbam3 = CBAMBlock(channels=128)
        self.cbam4 = CBAMBlock(channels=256)

        self.projection = nn.Sequential(
            nn.AdaptiveAvgPool2d(1),
            nn.Conv2d(256, latent_dims, kernel_size=1),
            nn.Flatten(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.dim() == 3:
            x = x.unsqueeze(1)
        x = self.act(self.conv1(x))
        x = self.act(self.conv2(x))
        x = self.act(self.conv3(x))
        x = self.cbam3(x)
        x = self.act(self.conv4(x))
        x = self.cbam4(x)
        return self.projection(x)


class CNNGRUFusionBackbone(nn.Module):
    """
    Fuse state + image latent, then process with GRU.
    Keeps an internal hidden state for step-by-step rollout.
    """
    def __init__(
        self,
        state_dim: int,
        img_latent_dims: int,
        output_dim: int,
        gru_input_dim: int = 256,
        gru_hidden_dim: int = 128,
        recurrent_detach: bool = True,
    ):
        super().__init__()
        self.gru_hidden_dim = int(gru_hidden_dim)
        self.recurrent_detach = bool(recurrent_detach)
        self._hidden_state = None

        self.fusion = nn.Sequential(
            nn.Linear(state_dim + img_latent_dims, 512),
            nn.ELU(),
            nn.Linear(512, gru_input_dim),
            nn.ELU(),
        )
        self.gru = nn.GRU(
            input_size=gru_input_dim,
            hidden_size=self.gru_hidden_dim,
            batch_first=True,
        )
        self.head = nn.Sequential(
            nn.Linear(self.gru_hidden_dim, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, output_dim),
        )

    def reset_hidden(self, batch_size: int = None, device=None, dtype=None) -> None:
        if batch_size is None:
            self._hidden_state = None
            return
        if device is None:
            device = next(self.parameters()).device
        if dtype is None:
            dtype = next(self.parameters()).dtype
        self._hidden_state = torch.zeros(
            1, int(batch_size), self.gru_hidden_dim, device=device, dtype=dtype
        )

    def _align_hidden(self, x: torch.Tensor):
        if self._hidden_state is None:
            return None
        if self._hidden_state.shape[1] != x.shape[0]:
            return None
        if self._hidden_state.device != x.device:
            return None
        if self._hidden_state.dtype != x.dtype:
            return None
        return self._hidden_state

    def forward(self, fused_input: torch.Tensor) -> torch.Tensor:
        x = self.fusion(fused_input)
        x = x.unsqueeze(1)  # [B, 1, D]

        hidden = self._align_hidden(x)
        out, next_hidden = self.gru(x, hidden)

        if self.recurrent_detach:
            self._hidden_state = next_hidden.detach()
        else:
            self._hidden_state = next_hidden

        return self.head(out.squeeze(1))


class CNNMLPFusionBackbone(nn.Module):
    """
    Fuse state + image latent, then process with pure MLP head.
    This matches the earlier state+CNN+MLP design for ablation/compare.
    """
    def __init__(self, state_dim: int, img_latent_dims: int, output_dim: int):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(state_dim + img_latent_dims, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, output_dim),
        )

    def reset_hidden(self, batch_size: int = None, device=None, dtype=None) -> None:
        # Kept for API compatibility with GRU policies.
        return None

    def forward(self, fused_input: torch.Tensor) -> torch.Tensor:
        return self.mlp(fused_input)


# =========================================================
# 3) State + Vision Fusion Policy (keep this)
# =========================================================

class StateVisionPolicy(nn.Module):
    """
    State + vision features fusion policy.
    - state input is normalized by RunningNorm (optional)
    - image input normalized by ImageNormalizer (optional mean/std)
    Output constraint matches TrackVel.
    """
    def __init__(
        self,
        state_dim: int,
        output_dim: int,
        img_size=(128, 128),
        img_channels: int = 1,
        img_latent_dims: int = 64,
        use_state_norm: bool = True,
        img_mean=None,
        img_std=None,
        gru_input_dim: int = 256,
        gru_hidden_dim: int = 128,
        recurrent_detach: bool = True,
        aux_orientation_dim: int = 0,
    ):
        super().__init__()
        self.use_state_norm = use_state_norm
        self.state_norm = RunningNorm(state_dim) if use_state_norm else None

        self.img_norm = ImageNormalizer(mean=img_mean, std=img_std)
        self.image_encoder = CNNImageEncoder(image_res=img_size, in_channels=img_channels, latent_dims=img_latent_dims)
        self.network = CNNGRUFusionBackbone(
            state_dim=state_dim,
            img_latent_dims=img_latent_dims,
            output_dim=output_dim,
            gru_input_dim=gru_input_dim,
            gru_hidden_dim=gru_hidden_dim,
            recurrent_detach=recurrent_detach,
        )

        # Optional auxiliary head that regresses the gate orientation directly
        # from the image latent. Supervising this forces the CNN to extract
        # orientation from depth in parallel with control, so the policy does
        # not collapse when a privileged orientation input is faded out.
        self.aux_orientation_dim = int(aux_orientation_dim)
        if self.aux_orientation_dim > 0:
            self.aux_orientation_head = nn.Sequential(
                nn.Linear(img_latent_dims, 128),
                nn.ELU(),
                nn.Linear(128, self.aux_orientation_dim),
            )
        else:
            self.aux_orientation_head = None
        # Populated on every forward pass; read by the training loop.
        self.last_orientation_pred = None

    def reset_hidden(self, batch_size: int = None, device=None, dtype=None) -> None:
        self.network.reset_hidden(batch_size=batch_size, device=device, dtype=dtype)

    def forward(self, state: torch.Tensor, img: torch.Tensor, update_norm: bool = False) -> torch.Tensor:
        # state: [B,state_dim], img: [B,H,W] or [B,C,H,W]
        if self.use_state_norm:
            state = self.state_norm(state, update=update_norm)

        img = self.img_norm(img)
        img_latent = self.image_encoder(img)
        if self.aux_orientation_head is not None:
            self.last_orientation_pred = self.aux_orientation_head(img_latent)
        else:
            self.last_orientation_pred = None
        x = torch.cat([state, img_latent], dim=1)

        out = self.network(x)
        first = torch.sigmoid(out[:, 0:1])   # (0,1)
        rest = torch.tanh(out[:, 1:])        # (-1,1)
        return torch.cat((first, rest), dim=1)


class StateVisionGRUNoCBAMPolicy(StateVisionPolicy):
    """
    State + vision GRU policy using the plain CNNImageEncoder, without CBAM.
    This is an explicit alias for experiments that compare against
    StateVisionGRUSpatialAttentionPolicy while keeping the same public API.
    """
    pass


class StateVisionMLPPolicy(nn.Module):
    """
    State + vision fusion policy with MLP head (no recurrence).
    Interface intentionally matches StateVisionPolicy for easy swapping.
    """
    def __init__(
        self,
        state_dim: int,
        output_dim: int,
        img_size=(128, 128),
        img_channels: int = 1,
        img_latent_dims: int = 64,
        use_state_norm: bool = True,
        img_mean=None,
        img_std=None,
    ):
        super().__init__()
        self.use_state_norm = use_state_norm
        self.state_norm = RunningNorm(state_dim) if use_state_norm else None

        self.img_norm = ImageNormalizer(mean=img_mean, std=img_std)
        self.image_encoder = CNNImageEncoder(image_res=img_size, in_channels=img_channels, latent_dims=img_latent_dims)
        self.network = CNNMLPFusionBackbone(
            state_dim=state_dim,
            img_latent_dims=img_latent_dims,
            output_dim=output_dim,
        )

    def reset_hidden(self, batch_size: int = None, device=None, dtype=None) -> None:
        # No recurrent state in this policy.
        return None

    def forward(self, state: torch.Tensor, img: torch.Tensor, update_norm: bool = False) -> torch.Tensor:
        if self.use_state_norm:
            state = self.state_norm(state, update=update_norm)

        img = self.img_norm(img)
        img_latent = self.image_encoder(img)
        x = torch.cat([state, img_latent], dim=1)

        out = self.network(x)
        first = torch.sigmoid(out[:, 0:1])   # (0,1)
        rest = torch.tanh(out[:, 1:])        # (-1,1)
        return torch.cat((first, rest), dim=1)


class StateVisionGRUSpatialAttentionPolicy(nn.Module):
    """
    State + vision fusion policy using GRU and CBAM on the last two CNN layers.
    """
    def __init__(
        self,
        state_dim: int,
        output_dim: int,
        img_size=(128, 128),
        img_channels: int = 1,
        img_latent_dims: int = 64,
        use_state_norm: bool = True,
        img_mean=None,
        img_std=None,
        gru_input_dim: int = 256,
        gru_hidden_dim: int = 128,
        recurrent_detach: bool = True,
    ):
        super().__init__()
        print("HELOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOo")
        self.use_state_norm = use_state_norm
        self.state_norm = RunningNorm(state_dim) if use_state_norm else None

        self.img_norm = ImageNormalizer(mean=img_mean, std=img_std)
        self.image_encoder = CNNImageEncoderCBAMLastTwo(
            image_res=img_size,
            in_channels=img_channels,
            latent_dims=img_latent_dims,
        )
        self.network = CNNGRUFusionBackbone(
            state_dim=state_dim,
            img_latent_dims=img_latent_dims,
            output_dim=output_dim,
            gru_input_dim=gru_input_dim,
            gru_hidden_dim=gru_hidden_dim,
            recurrent_detach=recurrent_detach,
        )

    def reset_hidden(self, batch_size: int = None, device=None, dtype=None) -> None:
        self.network.reset_hidden(batch_size=batch_size, device=device, dtype=dtype)

    def forward(self, state: torch.Tensor, img: torch.Tensor, update_norm: bool = False) -> torch.Tensor:
        if self.use_state_norm:
            state = self.state_norm(state, update=update_norm)

        img = self.img_norm(img)
        img_latent = self.image_encoder(img)
        x = torch.cat([state, img_latent], dim=1)

        out = self.network(x)
        first = torch.sigmoid(out[:, 0:1])   # (0,1)
        rest = torch.tanh(out[:, 1:])        # (-1,1)
        return torch.cat((first, rest), dim=1)


# =========================================================
# 4) Pure Vision Policy (new)
# =========================================================

class VisionOnlyPolicy(nn.Module):
    """
    Pure vision -> action policy.
    Output constraint matches TrackVel.
    """
    def __init__(
        self,
        output_dim: int = 4,
        img_size=(128, 128),
        img_channels: int = 1,
        img_latent_dims: int = 128,
        img_mean=None,
        img_std=None,
    ):
        super().__init__()
        self.img_norm = ImageNormalizer(mean=img_mean, std=img_std)
        self.image_encoder = CNNImageEncoder(image_res=img_size, in_channels=img_channels, latent_dims=img_latent_dims)

        self.head = nn.Sequential(
            nn.Linear(img_latent_dims, 256),
            nn.ELU(),
            nn.Linear(256, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, output_dim),
        )

    def forward(self, img: torch.Tensor) -> torch.Tensor:
        img = self.img_norm(img)                 # [B,C,H,W]
        z = self.image_encoder(img)              # [B,img_latent_dims]
        out = self.head(z)                       # [B,4]
        first = torch.sigmoid(out[:, 0:1])       # (0,1)
        rest = torch.tanh(out[:, 1:])            # (-1,1)
        return torch.cat((first, rest), dim=1)


# =========================================================
# 5) Backwards-compatible wrapper: original PolicyNetwork name
#    (State + Vision fusion, but keeps your old calling style)
# =========================================================

class PolicyNetwork(nn.Module):
    """
    Compatible with your old signature:
        forward(x, depth_imgs=None)
    - If depth_imgs is provided: fuse state + vision.
    - If depth_imgs is None: uses zero vision latent (kept for compatibility).
      (Note: for training a true vision-guided policy, ALWAYS pass depth_imgs.)
    Output constraint matches TrackVel.
    """
    def __init__(
        self,
        input_dim: int,
        output_dim: int,
        img_size,
        img_channels: int = 1,
        img_latent_dims: int = 64,
        use_state_norm: bool = True,
        img_mean=None,
        img_std=None,
        gru_input_dim: int = 256,
        gru_hidden_dim: int = 128,
        recurrent_detach: bool = True,
        fusion_backbone: str = "gru",
    ):
        super().__init__()
        self.use_state_norm = use_state_norm
        self.state_norm = RunningNorm(input_dim) if use_state_norm else None

        self.img_norm = ImageNormalizer(mean=img_mean, std=img_std)
        fusion_backbone = str(fusion_backbone).lower()
        if fusion_backbone == "gru":
            self.image_encoder = CNNImageEncoder(
                image_res=img_size,
                in_channels=img_channels,
                latent_dims=img_latent_dims,
            )
            self.network = CNNGRUFusionBackbone(
                state_dim=input_dim,
                img_latent_dims=img_latent_dims,
                output_dim=output_dim,
                gru_input_dim=gru_input_dim,
                gru_hidden_dim=gru_hidden_dim,
                recurrent_detach=recurrent_detach,
            )
        elif fusion_backbone in ("gru_spatial_attention", "gru_cbam", "gru_sa"):
            self.image_encoder = CNNImageEncoderCBAMLastTwo(
                image_res=img_size,
                in_channels=img_channels,
                latent_dims=img_latent_dims,
            )
            self.network = CNNGRUFusionBackbone(
                state_dim=input_dim,
                img_latent_dims=img_latent_dims,
                output_dim=output_dim,
                gru_input_dim=gru_input_dim,
                gru_hidden_dim=gru_hidden_dim,
                recurrent_detach=recurrent_detach,
            )
        elif fusion_backbone == "mlp":
            self.image_encoder = CNNImageEncoder(
                image_res=img_size,
                in_channels=img_channels,
                latent_dims=img_latent_dims,
            )
            self.network = CNNMLPFusionBackbone(
                state_dim=input_dim,
                img_latent_dims=img_latent_dims,
                output_dim=output_dim,
            )
        else:
            raise ValueError(
                f"Unsupported fusion_backbone: {fusion_backbone}. "
                "Use 'gru', 'gru_spatial_attention', or 'mlp'."
            )

    def reset_hidden(self, batch_size: int = None, device=None, dtype=None) -> None:
        if hasattr(self.network, "reset_hidden"):
            self.network.reset_hidden(batch_size=batch_size, device=device, dtype=dtype)

    def forward(self, x: torch.Tensor, depth_imgs: torch.Tensor = None, update_norm: bool = False) -> torch.Tensor:
        # x: [B,input_dim]
        if self.use_state_norm:
            x = self.state_norm(x, update=update_norm)

        if depth_imgs is not None:
            depth_imgs = self.img_norm(depth_imgs)
            depth_latent = self.image_encoder(depth_imgs)  # [B,latent]
        else:
            depth_latent = torch.zeros(x.shape[0], self.image_encoder.latent_dims, device=x.device, dtype=x.dtype)

        x = torch.cat([x, depth_latent], dim=1)
        out = self.network(x)

        first = torch.sigmoid(out[:, 0:1])  # (0,1)
        rest = torch.tanh(out[:, 1:])       # (-1,1)
        return torch.cat((first, rest), dim=1)
