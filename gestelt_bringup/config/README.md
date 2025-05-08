# Quadrotor state
$$\mathbf{x}=[\mathbf{p},\mathbf{v},\mathbf{q}]^T$$
where:
$\mathbf{p}=[r_x,r_y,r_z]^T$ is the position of the quadrotor in the world frame
# MPC cost function
## terminal cost
## path cost 
~~$$ J= \mathbf{p}^TQ \mathbf{p}+2P\mathbf{p}+\mathbf{u}^TR\mathbf{u}$$
Note:
$\mathbf{p}$ is the quadrotor position, not the position error to the goal
Expand the $Q$ and $P$
$$ J =Q_{11}r_x^2 +Q_{22}r_y^2 +Q_{33}r_z^2 +P_{1}r_x+P_{2}r_y+P_{3}r_z$$
where $r_x$, $r_y$, $r_z$ are the position error in x, y, z direction respectively.~~

## before the traverse, the path cost is:
problem want to solve, how the learn the reference far from the center?


$$ J=-()

## traverse cost
