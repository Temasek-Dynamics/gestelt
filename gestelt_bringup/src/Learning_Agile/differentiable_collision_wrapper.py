import numpy as np
from juliacall import Main as jl, convert


## load packages
jl.seval('import Pkg') 
jl.seval('Pkg.instantiate()')
jl.seval('Pkg.add("StaticArrays")')
jl.seval('Pkg.add("LinearAlgebra")')
jl.seval('Pkg.add("MeshCat")')
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
jl.seval('import MeshCat as mc')

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
                                    quad_half_height,
                                    drone_state):
    
    line_centers_G=np.matmul(line_centers,R_gate.T) 
    width_gap=np.abs(line_centers_G[1,0]-line_centers_G[3,0])
    height_gap=np.abs(line_centers_G[0,2]-line_centers_G[2,2])
    
    prism_size=quad_half_height
    A=np.diag([quad_radius,quad_radius,quad_half_height])
    A_inv=np.linalg.inv(A)
    P=A_inv.T@A_inv
  
   
    # test=np.matmul(R_gate.T,line_centers[0,])
    # print('line_centers_G:',line_centers_G)

    prism_centers=np.zeros([4,3])

    # left and right prisms centers
    prism_centers[1,]=line_centers_G[1,]+np.array([ quad_half_height,0,0])
    prism_centers[3,]=line_centers_G[3,]+np.array([-quad_half_height,0,0])

    # up and down prisms centers
    prism_centers[0,]=line_centers_G[0,]+np.array([0,0, quad_radius])
    prism_centers[2,]=line_centers_G[2,]+np.array([0,0,-quad_radius])

    prism_centers_W=np.matmul(prism_centers,R_gate)
    
    
    # create rectangle walls with the desired size
   
    P_obs = [jl.dc.create_rect_prism(width_gap, 1.0, quad_radius*2)[0],
             jl.dc.create_rect_prism(quad_half_height*2, 1.0, height_gap)[0],
             jl.dc.create_rect_prism(width_gap, 1.0, quad_radius*2)[0],
             jl.dc.create_rect_prism(quad_half_height*2, 1.0, height_gap)[0]]

    
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
    

    #============================visualize=======================##
    # vis = jl.mc.Visualizer()
    # jl.mc.open(vis)  

    # jl.seval('vis->mc.setprop!(vis["/Lights/AmbientLight/<object>"], "intensity", 0.9)')(vis)
    # jl.seval('vis->mc.setprop!(vis["/Lights/PointLightPositiveX/<object>"], "intensity", 0.0)')(vis)
    # jl.seval('vis->mc.setprop!(vis["/Lights/FillLight/<object>"], "intensity", 0.25)')(vis)
    # jl.seval('vis->mc.setvisible!(vis["/Grid"],false)')(vis)
    # jl.seval('vis->mc.setvisible!(vis["/Axes"],false)')(vis)


    # jl.seval('(vis, Elli_drone)->dc.build_primitive!(vis, Elli_drone, Symbol("drone"); color = mc.RGBA(0.0, 0.0, 0.0, 1.0), α = 1.0)')(vis, Elli_drone)
    # jl.seval('(vis, Elli_drone)->dc.update_pose!(vis[Symbol("drone")],Elli_drone)')(vis,Elli_drone) 
    
    # for i in range(len(P_obs)):
    #     jl.seval('(i,vis,current_obs)->dc.build_primitive!(vis, current_obs, Symbol("P"*string(i)); α = 1.0)')(i,vis,P_obs[i])    
    #     jl.seval('(i,vis,current_obs)->dc.update_pose!(vis[Symbol("P"*string(i))],current_obs)')(i,vis,P_obs[i]) 
    #======================end of visualize=======================##
    
    # return min scaling α and gradient of α wrt configurations 
    dalpha_dstate_drone=np.zeros(10) # p,v,q
    penalty=0
    for i in range(4): # P_obs[1],P_obs[3] (left and right walls)

        # dalpha_i_dstate: drone_p,drone_q,ellipse_p,ellipse_q
        alpha_i, dalpha_i_dstate=jl.dc.proximity_gradient(Elli_drone,P_obs[i],verbose = False, pdip_tol = 1e-6)

        # print(alpha_i)

        dalpha_i_dstate_np=np.array(dalpha_i_dstate.data)
        # print(dalpha_i_dstate_np)

        # penalty += 100 * np.log((alpha_i)) * (3/(alpha_i)**3) 
        # dalpha_dstate_drone[0:3] += (100/alpha_i * (3/(alpha_i)**3) + 100 * np.log((alpha_i)) * (-9/(alpha_i)**4)) * dalpha_i_dstate_np[0:3]
        # dalpha_dstate_drone[6:10] += (100/alpha_i * (3/(alpha_i)**3) + 100 * np.log((alpha_i)) * (-9/(alpha_i)**4)) * dalpha_i_dstate_np[3:7]
        
        scaling_w=100
        penalty+=(scaling_w*(alpha_i-1.2)**2)
        dalpha_dstate_drone[0:3] += 2 * scaling_w * (alpha_i-1.2) * dalpha_i_dstate_np[0:3]
        dalpha_dstate_drone[6:10] += 2 * scaling_w *(alpha_i-1.2) * dalpha_i_dstate_np[3:7]

    
    return penalty,dalpha_dstate_drone






