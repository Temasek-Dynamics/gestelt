import numpy as np
from juliacall import Main as jl, convert

# 请确保在 Julia 环境中这些包已经正确加载

## load packages
jl.seval('import Pkg') 
jl.seval('Pkg.add("StaticArrays")')
jl.seval('Pkg.add("LinearAlgebra")')
# jl.seval('Pkg.add(path="/home/tlab-uav/DifferentiableCollisions.jl")')
jl.seval('Pkg.add("DifferentiableCollisions")')

## necessary for the package update
jl.seval('Pkg.develop(path="/home/tlab-uav/DifferentiableCollisions.jl")')
jl.seval('Pkg.add("Revise")')

jl.seval('println("Hello, Julia!")')

## import packages
jl.seval('using Revise')
jl.seval('using LinearAlgebra')
jl.seval('using StaticArrays')
jl.seval('import DifferentiableCollisions as dc')

def dir_cosine_np(q):  # world frame to body frame
    C_B_I = np.array([
        [1 - 2 * (q[2] ** 2 + q[3] ** 2), 2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2])],
        [2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * (q[1] ** 2 + q[3] ** 2), 2 * (q[2] * q[3] + q[0] * q[1])],
        [2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)]
    ])
    return C_B_I

def DifferentiableCollisionsWrapper(line_centers,
                                    R_gate,
                                    gate_quat,
                                    quad_radius,
                                    quad_height,
                                    drone_state):

        
    prism_size=5
    A=np.diag([quad_radius,quad_radius,quad_height])
    A_inv=np.linalg.inv(A)
    P=A_inv.T@A_inv
  
    line_centers_G=np.matmul(line_centers,R_gate.T) 
    # test=np.matmul(R_gate.T,line_centers[0,])
    # print('line_centers_G:',line_centers_G)

    prism_centers=np.zeros([4,3])
    prism_centers[1,]=line_centers_G[1,]+np.array([ prism_size/2,0,0])
    prism_centers[3,]=line_centers_G[3,]+np.array([-prism_size/2,0,0])

    prism_centers_W=np.matmul(prism_centers,R_gate)


    # create rectangle walls with the desired size
   
    P_obs = [jl.dc.create_rect_prism(prism_size, 1.0, prism_size*2)[0],
            jl.dc.create_rect_prism(prism_size, 1.0, prism_size*2)[0],
            jl.dc.create_rect_prism(prism_size, 1.0, prism_size*2)[0],
            jl.dc.create_rect_prism(prism_size, 1.0, prism_size*2)[0]]

    
    P =jl.convert(jl.SMatrix[3,3,jl.Float64,9], P)
    Elli_drone = jl.dc.Ellipsoid(P) 
    
    ## assign pose of the prisms and ellipsoid
    # covert from numpy to Julia SVectors
    jl_q_prism=jl.convert(jl.SVector[4,jl.Float64], gate_quat)
    jl_q_drone=jl.convert(jl.SVector[4,jl.Float64], drone_state[6:10])

   
    P_obs[0].r = prism_centers_W[0,]
    P_obs[0].q = jl_q_prism
    
    P_obs[1].r = prism_centers_W[1,]
    P_obs[1].q = jl_q_prism

    P_obs[2].r = prism_centers_W[2,]
    P_obs[2].q = jl_q_prism

    P_obs[3].r = prism_centers_W[3,]
    P_obs[3].q = jl_q_prism

    Elli_drone.r = drone_state[0:3]
    Elli_drone.q = jl_q_drone
    
        

    # return min scaling α and gradient of α wrt configurations 
    dalpha_dstate_drone=np.zeros(10) # p,v,q
    reward=0
    reward_w=100
    for i in [1,3]: # P_obs[1],P_obs[3]

        # dalpha_i_dstate: drone_p,drone_q,ellipse_p,ellipse_q
        alpha_i, dalpha_i_dstate=jl.dc.proximity_gradient(Elli_drone,P_obs[i],verbose = False, pdip_tol = 1e-6)

        # print(alpha_i)

        dalpha_i_dstate_np=np.array(dalpha_i_dstate.data)
        # print(dalpha_i_dstate_np)

        reward += 3*np.log((alpha_i))
        dalpha_dstate_drone[0:3] += 3/alpha_i * dalpha_i_dstate_np[0:3]
        dalpha_dstate_drone[6:10] += 3/alpha_i * dalpha_i_dstate_np[3:7]


    ## keep at the center reward
    # reward_center = -2*(np.linalg.norm(drone_state[0:3]-np.array([0,0,0])))
    # reward+=reward_center

    return reward,dalpha_dstate_drone






