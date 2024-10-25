import numpy as np
from juliacall import Main as jl, convert
# 定义 Polytope 和 StaticArrays
# 请确保在 Julia 环境中这些包已经正确加载

## load packages
jl.seval('import Pkg') 
jl.seval('Pkg.update()')
jl.seval('Pkg.add("Revise")')
jl.seval('Pkg.add("StaticArrays")')
jl.seval('Pkg.add("LinearAlgebra")')
jl.seval('Pkg.add("DifferentiableCollisions")')
jl.seval('Pkg.develop(path="/home/tlab-uav/DifferentiableCollisions.jl")')
jl.seval('println("Hello, Julia!")')

## import packages
jl.seval('using Revise')
jl.seval('using LinearAlgebra')
jl.seval('using StaticArrays')
jl.seval('import DifferentiableCollisions as dc')

def julia_test():

    test=jl.dc.Sphere(1.0)
    test.r = np.array([4.0, 1.0, 0.0])
    
    P = np.diag([1.0, 1.0, 1.0])
    P =jl.convert(jl.SMatrix[3,3,jl.Float64], P)
    ellipsoid = jl.dc.Ellipsoid(P)

    test2=jl.dc.SphereMRP(2.0)
    test2.r=np.array([0.0, 1.0, 0.0])
    test2.r._jl_display()

    test3=jl.dc.CylinderMRP(1.0, 1.0)
    test3.r=np.array([0.0, 1.0, 0.0])
    test3.r._jl_display()
    # create rectangle walls
    
    # covert from numpy to Julia SVectors
    jl_q_svector=jl.convert(jl.SVector[4,jl.Float64], np.array([np.cos(np.pi/4), np.sin(np.pi/4),0, 0]))
    
    ellipsoid.q = jl_q_svector
    
    P_obs = [jl.dc.create_rect_prism(10, 10, 1)[0],
              jl.dc.create_rect_prism(10, 10, 1)[0],
              jl.dc.create_rect_prism(4, 4, 1)[0],
              jl.dc.create_rect_prism(4, 4, 1)[0]]
               # 10x10x1 prism
    
    
    P_obs[0].r = np.array([-6.0, 0.0, 5.0])
    P_obs[0].q = jl_q_svector
    
    P_obs[1].r = np.array([6.0, 0.0, 5.0])
    P_obs[1].q = jl_q_svector

    P_obs[2].r = np.array([0.0, 0.0, 2.05])
    P_obs[2].q = jl_q_svector

    P_obs[3].r = np.array([0.0, 0.0, 7.96])
    P_obs[3].q = jl_q_svector

    print(P_obs[0])

        
    # return min scaling α and intersection x
    # only MRP and MRP could compute the intersection
    alpha,x=jl.dc.proximity(ellipsoid,P_obs[0])

    print(alpha)
    # print(x)

    # return min scaling α and gradient of α wrt configurations 
    # alpha, dalpha_dstate=jl.dc.proximity_gradient(test,P_obs[0],verbose = False, pdip_tol = 1e-6)
    

    # Julia Tuple automatically converted to Python tuple,
    # then here we convert the tuple to numpy array
    # data=np.array(dalpha_dstate.data)

    # return min scaling α, intersection x, and jacobian J (*)
    # alpha,x,J=jl.dc.proximity_jacobian(test,test2,verbose = False, pdip_tol = 1e-6)
    
    # J_np = convert(np.array, J.data)

    # pass
    # print(alpha)
    # print(x)
    # print(J)

if __name__ == '__main__':
    julia_test()