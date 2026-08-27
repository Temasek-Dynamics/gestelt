import torch
# residual_dyn_train.py
import pickle
import math
import numpy as np
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
import pandas as pd
from typing import Dict, List, Tuple
import os

def skew(v: torch.Tensor) -> torch.Tensor:
    """
    Compute skew-symmetric matrix for a vector.
    v: (...,3)
    returns: (...,3,3)
    """
    zero = torch.zeros_like(v[..., 0])
    vx, vy, vz = v[..., 0], v[..., 1], v[..., 2]
    K = torch.stack([
        torch.stack([zero, -vz, vy], dim=-1),
        torch.stack([vz, zero, -vx], dim=-1),
        torch.stack([-vy, vx, zero], dim=-1)
    ], dim=-2)
    return K

def rotation_matrix_from_vector(v: torch.Tensor) -> torch.Tensor:
    """
    Rodrigues formula (PyTorch) with eps for small angles.
    v: (...,3) rotation vector
    returns: (...,3,3) rotation matrix
    """
    eps = 1e-5
    K = skew(v)
    theta = torch.linalg.norm(torch.abs(v) + eps, dim=-1, keepdim=True)  # (...,1)
    theta2 = theta * theta
    I = torch.eye(3, device=v.device, dtype=v.dtype).expand(list(v.shape[:-1]) + [3, 3])
    K2 = K @ K

    sin_term = torch.sin(theta) / theta
    cos_term = (1 - torch.cos(theta)) / theta2
    sin_term = sin_term.view(*([*v.shape[:-1], 1, 1]))
    cos_term = cos_term.view(*([*v.shape[:-1], 1, 1]))

    R = I + sin_term * K + cos_term * K2
    return R

def quadrotor_dyn_torch(p: torch.Tensor, R: torch.Tensor, v: torch.Tensor,
                        a: torch.Tensor, omega: torch.Tensor, dt: torch.Tensor,
                        gravity: torch.Tensor = None) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Simple quadrotor dynamics (Euler step for position/velocity, exact for rotation)
    p: (...,3)
    R: (...,3,3)
    v: (...,3)
    a: scalar or (...,)
    omega: (...,3)
    dt: scalar or (...,)
    gravity: (...,3)
    returns: p_new, R_new, v_new
    """
    if gravity is None:
        gravity = torch.tensor([0., 0., -9.81], device=p.device, dtype=p.dtype)

    # 1) Compute world-frame acceleration
    # thrust along body z-axis
    if p.ndim == 1:  # single quad
        thrust_body = torch.tensor([0., 0., a], dtype=p.dtype, device=p.device)
        accel_world = gravity + R @ thrust_body
        p_new = p + dt * v
        v_new = v + dt * accel_world
        R_delta = rotation_matrix_from_vector(dt * omega)
        R_new = R @ R_delta
    else:  # batch of quads
        thrust_body = torch.zeros_like(p)  # (B,3)
        thrust_body[:, 2] = a
        accel_world = gravity + torch.bmm(R, thrust_body.unsqueeze(-1)).squeeze(-1)
        p_new = p + dt * v
        v_new = v + dt * accel_world
        R_delta = rotation_matrix_from_vector(dt * omega)
        R_new = torch.matmul(R, R_delta)

    return p_new, R_new, v_new, accel_world



# ----------------------------
# Dataset: form (state_t, action_t, dt) -> state_{t+1}
# ----------------------------
class ResidualDynamicsDataset(Dataset):
    def __init__(self, data_store: Dict,device='cpu'):
        """
        data_store: dict keyed by timestamps -> dict with keys:
          "position": [3],
          "rotation_matrix": [3,3] OR flattened 9,
          "velocity": [3],
          "action": array like [f, wx, wy, wz] OR dict, adjust below
        """
        self.device = device
        # Sort timestamps
        times = sorted(data_store.keys())
        self.examples = []  # list of (t, t_next)
        for i in range(len(times)-1):
            t, tn = times[i], times[i+1]
            # require dt > 0
            dt = tn - t
            if dt <= 0:
                continue
            # store pairs if both entries are valid
            s = data_store[t]
            sn = data_store[tn]
            # basic validation
            required = ["position", "rotation_matrix", "velocity", "action", "lin_acc"]
            if all(k in s for k in required) and all(k in sn for k in required):
                self.examples.append((t, tn))
        self.data_store = data_store

    def __len__(self):
        return len(self.examples)

    def _read(self, t):
        e = self.data_store[t]
        # position
        p = np.asarray(e["position"], dtype=np.float32)
        v = np.asarray(e["velocity"], dtype=np.float32)
        # rotation: might be flattened or 3x3
        R = np.asarray(e["rotation_matrix"], dtype=np.float32)
        if R.shape == (9,):
            R = R.reshape(3,3)
        # action: assume action = [f, omega_x, omega_y, omega_z]
        action = np.asarray(e["action"], dtype=np.float32)
        lin_acc = np.asarray(e["lin_acc"], dtype=np.float32)
        return p, v, R, action, lin_acc

    def __getitem__(self, idx):
        t, tn = self.examples[idx]
        dt = float(tn - t)
        p, v, R, action, _ = self._read(t)
        # p_n, v_n, R_n, _ = None, None, None, None
        _, _, _, _, lin_acc = self._read(tn)  #lin_acc is mass normalized acc - gravity already in world frame
        # convert to torch
        p_t = torch.from_numpy(p)
        v_t = torch.from_numpy(v)
        R_t = torch.from_numpy(R)
        # action: first element is thrust f, rest is body rates omega (3)
        f = action[0]
        omega = action[1:4] if action.shape[0] >= 4 else np.zeros(3, dtype=np.float32)
        f_t = torch.tensor(f, dtype=torch.float32)
        omega_t = torch.from_numpy(omega)
        l_a = torch.from_numpy(lin_acc)
        sample = {
            "p": p_t.float(),
            "v": v_t.float(),
            "R": R_t.float(),
            "f": f_t.float(),
            "omega": omega_t.float(),
            "lin_acc": l_a.float(),
        }
        return sample

# ----------------------------
# Residual MLP
# ----------------------------
class ResidualNet(nn.Module):
    def __init__(self, in_size: int, hidden: int = 256, out_size: int = 3, nl=nn.ReLU):
        """
        in_size: dimension of flattened inputs (p(3)+R(9)+v(3)+f(1)+omega(3)+dt(1) = 20)
        out_size: predict residual for [delta_p(3), delta_v(3)] -> 6
        """
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_size, hidden),
            nl(),
            nn.LayerNorm(hidden),
            nn.Linear(hidden, hidden),
            nl(),
            nn.Linear(hidden, out_size)
        )

    def forward(self, x):
        return self.net(x)

# ----------------------------
# Training helper: collate and compute base prediction + residual target
# ----------------------------
def collate_fn(batch):
    # combine into batched tensors
    p = torch.stack([b["p"] for b in batch], dim=0)
    v = torch.stack([b["v"] for b in batch], dim=0)
    R = torch.stack([b["R"] for b in batch], dim=0)
    f = torch.stack([b["f"] for b in batch], dim=0)
    omega = torch.stack([b["omega"] for b in batch], dim=0)
    lin_acc = torch.stack([b["lin_acc"] for b in batch], dim=0)

    return {"p":p, "v":v, "R":R, "f":f, "omega":omega, "lin_acc":lin_acc}

# ----------------------------
# Full training loop
# ----------------------------
def train(data_store_path: str,
          model_save_path: str = "residual_model.pt",
          epochs: int = 30,
          batch_size: int = 64,
          lr: float = 1e-3,
          device: str = None,
          drone_mass = 0.234):

    if device is None:
        device = "cuda" if torch.cuda.is_available() else "cpu"
    print("device:", device)

    # load pickle
    with open(data_store_path, "rb") as f:
        data_store = pickle.load(f)

    ds = ResidualDynamicsDataset(data_store)
    loader = DataLoader(ds, batch_size=batch_size, shuffle=True, collate_fn=collate_fn)

    # input dims: p(3) + R(9) + v(3) + f(1) + omega(3) + dt(1) = 20
    in_dim = 3 + 9 + 3 + 1 + 3
    model = ResidualNet(in_size=in_dim, hidden=256, out_size=3).to(device)
    optim = torch.optim.Adam(model.parameters(), lr=lr)
    scheduler = torch.optim.lr_scheduler.StepLR(optim, step_size=10, gamma=0.5)
    mse = nn.MSELoss()

    for ep in range(epochs):
        model.train()
        epoch_loss = 0.0
        for batch in loader:
            p = batch["p"].to(device)
            v = batch["v"].to(device)
            R = batch["R"].to(device)
            f = batch["f"].to(device)
            omega = batch["omega"].to(device)
            lin_acc = batch["lin_acc"].to(device)

            # base prediction
            a = (f / drone_mass).to(device)  # This is body mass_normalized acceleration.
            _, _, _, accel_world = quadrotor_dyn_torch(p, R, v, a, omega, 0.02)

            # target residuals
            # residual for p_next and v_next
            res_lin_acc = lin_acc - accel_world  # (...,6)

            # build input features
            R_flat = R.view(R.shape[0], -1)
            inp = torch.cat([p, R_flat, v, a.view(-1,1), omega], dim=-1)  # (B, in_dim)

            pred_res = model(inp)  # (B,6)
            loss = mse(pred_res, res_lin_acc)

            optim.zero_grad()
            loss.backward()
            optim.step()

            epoch_loss += loss.item() * p.shape[0]

        scheduler.step()
        epoch_loss /= len(ds)
        print(f"Epoch {ep+1}/{epochs} loss: {epoch_loss:.6f}")

    # save model and optionally scaler / metadata
    torch.save({
        "model_state": model.state_dict(),
        "in_dim": in_dim,
        "mass": drone_mass
    }, model_save_path)
    print("Saved model to", model_save_path)

# def main():
#     # Initialize single quadrotor state
#     p = torch.tensor([1.0, -2.0, 0.5], dtype=torch.float32)
#     v = torch.tensor([0.5, -0.3, 0.2], dtype=torch.float32)
#     R = torch.tensor([[0.0, -1.0, 0.0],
#                     [1.0,  0.0, 0.0],
#                     [0.0,  0.0, 1.0]], dtype=torch.float32)

#     # Dynamics inputs
#     a = 7.0 / 0.752             # thrust / mass
#     omega = torch.tensor([0.1, 0.2, 0.3], dtype=torch.float32)
#     dt = torch.tensor(0.01, dtype=torch.float32)
#     gravity = torch.tensor([0.0, 0.0, -9.81], dtype=torch.float32)

#     # Step dynamics
#     p_new, R_new, v_new = quadrotor_dyn_torch(p, R, v, a, omega, dt, gravity)

#     # Print results
#     print("Old position:", p)
#     print("New position:", p_new)
#     print("Old velocity:", v)
#     print("New velocity:", v_new)
#     print("Old rotation matrix:\n", R)
#     print("New rotation matrix:\n", R_new)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    # parser.add_argument("--data", type=str, default="recorded_data.pkl")
    # parser.add_argument("--out", type=str, default="residual_model.pt")
    file_num = 6
    parser.add_argument("--epochs", type=int, default=500)
    parser.add_argument("--batch", type=int, default=128)
    args = parser.parse_args()
    load_directory = "/home/yanrui/tempstorage4/rpg_flightning/data"
    files = [f for f in os.listdir(load_directory) if os.path.isfile(os.path.join(load_directory, f))]
    total_files = len(files) - 1
    full_load_path = os.path.join(load_directory, "data_collected_" + str(file_num) + ".pkl")
    full_save_path = os.path.join(load_directory, "model_" + str(file_num) + ".pt")

    train(full_load_path, model_save_path=full_save_path, epochs=args.epochs, batch_size=args.batch, drone_mass = 0.234)
    # main()

