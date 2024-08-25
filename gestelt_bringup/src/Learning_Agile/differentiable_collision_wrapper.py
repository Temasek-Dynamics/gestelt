import numpy as np
# from juliacall import Main as jl, convert
# # 定义 Polytope 和 StaticArrays
# # 请确保在 Julia 环境中这些包已经正确加载

# ## load packages
# jl.seval('import Pkg') 
# jl.seval('Pkg.add("StaticArrays")')
# jl.seval('Pkg.add("LinearAlgebra")')
# jl.seval('Pkg.add("DifferentiableCollisions")')
# jl.seval('println("Hello, Julia!")')

# ## import packages
# jl.seval('using LinearAlgebra')
# jl.seval('using StaticArrays')
# jl.seval('import DifferentiableCollisions as dc')


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
                                    drone_state):
    prism_size=10
   
  
    line_centers_G=np.matmul(line_centers,R_gate.T) 
    # test=np.matmul(R_gate.T,line_centers[0,])
    # print('line_centers_G:',line_centers_G)

    prism_centers=np.zeros([4,3])
    prism_centers[1,]=line_centers_G[1,]+np.array([ prism_size/2,0,0])
    prism_centers[3,]=line_centers_G[3,]+np.array([-prism_size/2,0,0])

    prism_centers_W=np.matmul(prism_centers,R_gate)
   
    pass
    ###transform the line_center list to gate frame


#     # create rectangle walls
#     P_obs = [jl.dc.create_rect_prism(10, 10, 1)[0],
#               jl.dc.create_rect_prism(10, 10, 1)[0],
#               jl.dc.create_rect_prism(4, 4, 1)[0],
#               jl.dc.create_rect_prism(4, 4, 1)[0]]
#                # 10x10x1 prism
    
#     # covert from numpy to Julia SVectors
#     jl_q_svector=jl.convert(jl.SVector[4,jl.Float64], np.array([np.cos(np.pi/4), np.sin(np.pi/4),0, 0]))
    
#     P_obs[0].r = np.array([-6.0, 0.0, 5.0])
#     P_obs[0].p = jl.dc.mrp_from_q(jl_q_svector)
    
#     P_obs[1].r = np.array([6.0, 0.0, 5.0])
#     P_obs[1].p = jl.dc.mrp_from_q(jl_q_svector)

#     P_obs[2].r = np.array([0.0, 0.0, 2.05])
#     P_obs[2].p = jl.dc.mrp_from_q(jl_q_svector)

#     P_obs[3].r = np.array([0.0, 0.0, 7.96])
#     P_obs[3].p = jl.dc.mrp_from_q(jl_q_svector)

#     print(P_obs[0])

        
#     # return min scaling α and intersection x
#     alpha,x=jl.dc.proximity(test,test2,verbose = False, pdip_tol = 1e-6)

#     # print(alpha)
#     # print(x)

#     # return min scaling α and gradient of α wrt configurations 
#     alpha, dalpha_dstate=jl.dc.proximity_gradient(test,test2,verbose = False, pdip_tol = 1e-6)
#     # print(alpha)
#     # print(dalpha_dstate)

#     # return min scaling α, intersection x, and jacobian J (*)
#     alpha,x,J=jl.dc.proximity_jacobian(test,test2,verbose = False, pdip_tol = 1e-6)
#     # print(alpha)
#     # print(x)
#     # print(J)
# if __name__ == '__main__':
#     DifferentiableCollisionsWrapper()


#############backup################ 
# def DifferentiableCollisionsWrapper(line_center,
#                                     gate_pitch,
#                                     drone_state):
#     test=jl.dc.Sphere(1)
#     test.r = np.array([2.0, 1.0, 0.0])
#     # test.r._jl_display()

#     test2=jl.dc.Sphere(2)
#     test2.r=np.array([0.0, 1.0, 0.0])
#     test2.r._jl_display()


#     # create rectangle walls
#     P_obs = [jl.dc.create_rect_prism(10, 10, 1)[0],
#               jl.dc.create_rect_prism(10, 10, 1)[0],
#               jl.dc.create_rect_prism(4, 4, 1)[0],
#               jl.dc.create_rect_prism(4, 4, 1)[0]]
#                # 10x10x1 prism
    
#     # covert from numpy to Julia SVectors
#     jl_q_svector=jl.convert(jl.SVector[4,jl.Float64], np.array([np.cos(np.pi/4), np.sin(np.pi/4),0, 0]))
    
#     P_obs[0].r = np.array([-6.0, 0.0, 5.0])
#     P_obs[0].p = jl.dc.mrp_from_q(jl_q_svector)
    
#     P_obs[1].r = np.array([6.0, 0.0, 5.0])
#     P_obs[1].p = jl.dc.mrp_from_q(jl_q_svector)

#     P_obs[2].r = np.array([0.0, 0.0, 2.05])
#     P_obs[2].p = jl.dc.mrp_from_q(jl_q_svector)

#     P_obs[3].r = np.array([0.0, 0.0, 7.96])
#     P_obs[3].p = jl.dc.mrp_from_q(jl_q_svector)

#     print(P_obs[0])

        
#     # return min scaling α and intersection x
#     alpha,x=jl.dc.proximity(test,test2,verbose = False, pdip_tol = 1e-6)

#     # print(alpha)
#     # print(x)

#     # return min scaling α and gradient of α wrt configurations 
#     alpha, dalpha_dstate=jl.dc.proximity_gradient(test,test2,verbose = False, pdip_tol = 1e-6)
#     # print(alpha)
#     # print(dalpha_dstate)

#     # return min scaling α, intersection x, and jacobian J (*)
#     alpha,x,J=jl.dc.proximity_jacobian(test,test2,verbose = False, pdip_tol = 1e-6)
#     # print(alpha)
#     # print(x)
#     # print(J)