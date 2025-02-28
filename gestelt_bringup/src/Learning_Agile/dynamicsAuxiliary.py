"""
This file contains the method that supervises the NN output for dynamically feasible high-level decisions.
"""
## get the current working directory
import os
import sys
cwd = os.path.dirname(os.path.abspath(__file__))
## add the path to the sys.path
build_path = os.path.abspath(os.path.join(cwd, "../../../../../build/mppi_py_interface"))
sys.path.append(build_path)
import torch
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mppi_py_interface
from torch_model import quadrotor_dynamics
class MPPI:
    def __init__(self, dynamics,dt, cost_fn, state_dim, action_dim, horizon=10, num_samples=1000, lambda_=1.0, noise_sigma=0.1, device="cuda"):
        """
        Model Predictive Path Integral (MPPI) controller using PyTorch for parallel computation.

        :param dynamics: System dynamics function f(x, u), must be a PyTorch function.
        :param dt: Time step for dynamics integration.
        :param cost_fn: Cost function c(x, u), must be a PyTorch function.
        :param state_dim: Dimensionality of the system state.
        :param action_dim: Dimensionality of the control input.
        :param horizon: Prediction horizon (T).
        :param num_samples: Number of sampled control trajectories (K).
        :param lambda_: Temperature parameter for softmax weighting.
        :param noise_sigma: Standard deviation of the control noise.
        :param device: Computation device ("cuda" or "cpu").
        """
        self.dynamics = dynamics
        self.dt = dt
        self.cost_fn = cost_fn
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.horizon = horizon
        self.num_samples = num_samples
        self.lambda_ = lambda_
        self.noise_sigma = noise_sigma
        self.device = torch.device(device)

        # Initialize the nominal control sequence
        self.u_seq = torch.zeros((horizon, action_dim), device=self.device)
        self.u_seq[:, 0] = 10.0  # Initial thrust
        self.u_seq[:, 1] = 5.0  # Initial angular rate x
        self.u_seq[:, 2] = 0.0  # Initial angular rate y

    def sample_trajectories(self, x):
        """
        Sample control trajectories and compute their costs in parallel.

        :param x: Current system state (torch tensor of shape [state_dim]).
        :return: Optimal control input u_0 to be executed.
        """
        # Expand state for parallel computation (shape: [num_samples, state_dim])
        x = x.unsqueeze(0).expand(self.num_samples, -1).to(self.device)

        # Generate random control noise (shape: [num_samples, horizon, action_dim])
        noise = torch.randn((self.num_samples, self.horizon, self.action_dim), device=self.device) * self.noise_sigma

        # Define mean and standard deviation for noise
        # mean = torch.zeros((self.num_samples, self.horizon, self.action_dim), device=self.device)  # Mean (all zeros)
        # std = torch.full((self.num_samples, self.horizon, self.action_dim), self.noise_sigma, device=self.device)  # Std (uniform sigma)

        # Generate noise with specific mean and std
        # noise = torch.normal(mean, std)


        # Generate control samples (shape: [num_samples, horizon, action_dim])
        u_samples = self.u_seq.unsqueeze(0) + noise 

        # Initialize cost tensor
        costs = torch.zeros(self.num_samples, device=self.device)

        # Simulate system dynamics and compute cost for each trajectory
        x_samples = x.clone()
        trajectory_samples = torch.zeros((self.num_samples, self.horizon, self.state_dim), device=self.device)

        for t in range(self.horizon):
            x_samples = self.dynamics(x_samples, u_samples[:, t, :])  # Compute next state
            costs += self.cost_fn(x_samples, u_samples[:, t, :])  # Accumulate cost
            trajectory_samples[:, t, :] = x_samples  # Store trajectory

        # Compute weights using softmax function
        min_cost = torch.min(costs)
        weights = torch.exp(- (costs - min_cost) / self.lambda_)
        weights /= torch.sum(weights)

        # Update the control sequence using weighted sum
        self.u_seq = torch.sum(weights[:, None, None] * (u_samples + noise), dim=0)

        # Get the optimal trajectory
        best_traj_idx = torch.argmax(weights)
        best_trajectory = trajectory_samples[best_traj_idx].cpu().numpy()

        return self.u_seq[0], best_trajectory, trajectory_samples.cpu().numpy()  # Return control, best trajectory, all sampled trajectories

def simple_dynamics(x, u):
    """
    Example dynamics: x_{t+1} = x_t + u_t
    :param x: Current state [num_samples, state_dim]
    :param u: Control input [num_samples, action_dim]
    :return: Next state
    """
    return x + u  # Simple linear dynamics (can be replaced with actual drone/robot model)

def cost_function(x, u):
    """
    Quadratic cost function: Penalizes deviation from the target.
    :param x: Current state [num_samples, state_dim]
    :param u: Control input [num_samples, action_dim]
    :return: Cost [num_samples]
    """
    target = torch.tensor([1.0, 1.0, 1.0, 0.0, 0.0, 0.0,1.0, 0.0, 0.0,0.0], device=x.device)
    return 5000*torch.sum((x - target) ** 2, dim=1) + 10 * torch.sum(u ** 2, dim=1)


def main():
     # Create MPPI controller
    controller = MPPI(
        dynamics=quadrotor_dynamics,
        dt=0.05,
        cost_fn=cost_function,
        state_dim=10,  # 3D state
        action_dim=4,  # 3D control
        horizon=10,
        num_samples=1000,
        lambda_=1.0,
        noise_sigma=0.5,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )

    # Initialize state
    x = torch.tensor([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], device=controller.device)
    trajectory_history = [x.cpu().numpy()]

    # Visualization setup
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    goal = np.array([1, 1, 1])

    for i in range(80):  # Run for 20 time steps
        u, best_trajectory, trajectory_samples = controller.sample_trajectories(x)  # Compute optimal control
        x = x.unsqueeze(0)
        u = u.unsqueeze(0)
        x =quadrotor_dynamics(x, u,dt=0.05)  # Apply control to the system
        x = x.squeeze(0)
        trajectory_history.append(x.cpu().numpy().squeeze())

        # Clear the plot
        ax.cla()

        # Plot target point
        ax.scatter(goal[0], goal[1], goal[2], color='red', label="Target", s=100)

        # Plot actual trajectory
        trajectory_np = np.array(trajectory_history)
        ax.plot(trajectory_np[:, 0], trajectory_np[:, 1], trajectory_np[:, 2], "b.-", label="Actual Trajectory")

        # Plot predicted trajectory
        ax.plot(best_trajectory[:, 0], best_trajectory[:, 1], best_trajectory[:, 2], "g--", label="Predicted Trajectory")

        # Plot sampled trajectories
        # for i in range(0,trajectory_samples.shape[0],50):
        for i in range(trajectory_samples.shape[0]):
            ax.plot(trajectory_samples[i, :, 0], trajectory_samples[i, :, 1], trajectory_samples[i, :, 2], "c-", alpha=0.1)

        # Set 3D limits and labels
        ax.set_xlim(-0.2, 1.2)
        ax.set_ylim(-0.2, 1.2)
        ax.set_zlim(-0.2, 1.2)
        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_zlabel("Z Position")
        ax.legend()
        plt.pause(0.01)  # Pause for animation effect

    plt.show()

if __name__ == "__main__":
#    mppi_py_interface.add(1,2)
#    print(mppi_py_interface.add(1,2))
    main()