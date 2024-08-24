import numpy as np
from juliacall import Main as jl, convert
# 定义 Polytope 和 StaticArrays
# 请确保在 Julia 环境中这些包已经正确加载

## load packages
jl.seval('import Pkg') 
jl.seval('Pkg.add("StaticArrays")')
jl.seval('Pkg.add("LinearAlgebra")')
jl.seval('Pkg.add("DifferentiableCollisions")')
jl.seval('println("Hello, Julia!")')

## import packages
jl.seval('using LinearAlgebra')
jl.seval('using StaticArrays')
jl.seval('import DifferentiableCollisions as dc')






test=jl.dc.Sphere(1)
test.r = np.array([2.0, 1.0, 0.0])
# test.r._jl_display()

test2=jl.dc.Sphere(2)
test2.r=np.array([0.0, 1.0, 0.0])
test2.r._jl_display()

# return min scaling α and intersection x
alpha,x=jl.dc.proximity(test,test2,verbose = False, pdip_tol = 1e-6)

print(alpha)
print(x)

# return min scaling α and gradient of α wrt configurations 
alpha, dalpha_dstate=jl.dc.proximity_gradient(test,test2,verbose = False, pdip_tol = 1e-6)
print(alpha)
print(dalpha_dstate)