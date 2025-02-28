import torch

def skew(vec):
    """
    Compute the skew-symmetric matrix of a 3D vector.
    
    :param vec: 3D angular velocity vector (batch, 3)
    :return: 3x3 skew-symmetric matrix (batch, 3, 3)
    """
    batch_size = vec.shape[0]
    zero = torch.zeros(batch_size, device=vec.device)
    
    return torch.stack([
        torch.stack([zero, -vec[:, 2], vec[:, 1]], dim=1),
        torch.stack([vec[:, 2], zero, -vec[:, 0]], dim=1),
        torch.stack([-vec[:, 1], vec[:, 0], zero], dim=1)
    ], dim=1)

def dir_cosine(q):
    """
    Compute the direction cosine matrix (DCM) from quaternion.
    
    :param q: Quaternion (batch, 4) [qw, qx, qy, qz]
    :return: 3x3 rotation matrix (batch, 3, 3)
    """
    qw, qx, qy, qz = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    
    C_B_I = torch.stack([
        torch.stack([1 - 2 * (qy**2 + qz**2), 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy)], dim=1),
        torch.stack([2 * (qx*qy + qw*qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy*qz - qw*qx)], dim=1),
        torch.stack([2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 1 - 2 * (qx**2 + qy**2)], dim=1)
    ], dim=1)
    
    return C_B_I

def omega(ang_rate_B):
    """
    Compute quaternion rate matrix for given angular velocity.
    
    :param ang_rate_B: Angular velocity vector (batch, 3)
    :return: 4x4 Omega matrix (batch, 4, 4)
    """
    batch_size = ang_rate_B.shape[0]
    zero = torch.zeros(batch_size, device=ang_rate_B.device)
    
    return torch.stack([
        torch.stack([zero, -ang_rate_B[:, 0], -ang_rate_B[:, 1], -ang_rate_B[:, 2]], dim=1),
        torch.stack([ang_rate_B[:, 0], zero, ang_rate_B[:, 2], -ang_rate_B[:, 1]], dim=1),
        torch.stack([ang_rate_B[:, 1], -ang_rate_B[:, 2], zero, ang_rate_B[:, 0]], dim=1),
        torch.stack([ang_rate_B[:, 2], ang_rate_B[:, 1], -ang_rate_B[:, 0], zero], dim=1),
    ], dim=1)

def quadrotor_dynamics(x, u, dt=0.05):
    """
    Quadrotor dynamics based on Newton-Euler equations.

    :param x: State vector [px, py, pz, vx, vy, vz, qx, qy, qz]
    :param u: Control input [ thrust, angular_rate_x, angular_rate_y, angular_rate_z]
    :param dt: Time step
    :return: Next state
    """
    g = torch.tensor([0, 0, -9.81], device=x.device)  # Gravity (m/s^2)
    m = 1.0  # Quadrotor mass (kg)
    J_B = torch.diag(torch.tensor([0.02, 0.02, 0.04], device=x.device))  # Inertia matrix

    # Extract state variables
    pos = x[:, :3]  # Position [px, py, pz]
    vel = x[:, 3:6]  # Velocity [vx, vy, vz]
    quat = x[:, 6:10]  # Quaternion [qw, qx, qy, qz]

    # Control inputs
    ang_rate = u[:, 1:4]  # Angular rate commands
    thrust = u[:, 0].unsqueeze(-1)  # Collective thrust
    thrust_vec = torch.cat([torch.zeros_like(thrust), torch.zeros_like(thrust), thrust], dim=-1)
    # Compute direction cosine matrix (DCM)
    C_B_I = dir_cosine(quat)  # Inertial to body frame
    C_I_B = C_B_I.transpose(1, 2)  # Body to inertial frame

    # Newton’s law (translational dynamics)
    dr_I = vel
    dv_I = g + torch.matmul(C_I_B, thrust_vec.unsqueeze(-1)).squeeze(-1) / m

    # Euler’s law (rotational dynamics)
    dq = 0.5 * torch.matmul(omega(ang_rate), quat.unsqueeze(-1)).squeeze(-1)
    # dw = torch.matmul(torch.linalg.inv(J_B), ang_rate - torch.matmul(torch.matmul(skew(ang_rate), J_B), ang_rate.unsqueeze(-1)).squeeze(-1))

    # Integrate state variables
    new_pos = pos + dr_I * dt
    new_vel = vel + dv_I * dt
    new_quat =quat + dq * dt
  

    # Return next state
    return torch.cat([new_pos, new_vel, new_quat], dim=-1)


def RK4_integration(dynamics, x, u, dt):
    """
    Perform 4th order Runge-Kutta integration for a given dynamics function.

    :param dynamics: Function to compute the state derivative dx/dt.
    :param x: Current state.
    :param u: Control input.
    :param dt: Time step.
    :return: Next state.
    """
    k1 = dynamics(x, u)
    k2 = dynamics(x + 0.5 * k1 * dt, u)
    k3 = dynamics(x + 0.5 * k2 * dt, u)
    k4 = dynamics(x + k3 * dt, u)
    return x + (k1 + 2*k2 + 2*k3 + k4) * dt / 6