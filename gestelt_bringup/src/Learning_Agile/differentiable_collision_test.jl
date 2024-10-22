import Pkg
Pkg.activate(".")
# Pkg.add("StaticArrays")
# Pkg.add("MeshCat")
Pkg.add(path="/home/tlab-uav/DifferentiableCollisions.jl")
Pkg.develop(path="/home/tlab-uav/DifferentiableCollisions.jl")
Pkg.add("Revise")
import DifferentiableCollisions as dc
import MeshCat as mc
using StaticArrays
using LinearAlgebra
using Revise
# vis = mc.Visualizer()
# mc.open(vis)

# cone = dc.Cone(3.0, deg2rad(22))
# cone.r = @SVector randn(3)
# cone.q = normalize((@SVector randn(4)))

# # build primitive scaled by α = 1.0
# dc.build_primitive!(vis, cone, :cone; α = 1.0, color = mc.RGBA(1,0,0,1.0))

# # update position and attitude
# dc.update_pose!(vis[:cone], cone)

# P= diagm(SVector(1.0, 2.0, 3.0))
# print(P)
# ellipse = dc.EllipsoidMRP(P)
P1=dc.create_rect_prism(1,2,3)[1]
P2=dc.create_rect_prism(2,4,3)[1]
P1.r=SA[1,3,-2.]
P2.r=SA[-1,0.1,2.]
P1.q=SA[0.58730155, 0.15873018, 0.47619054, 0.6349207 ]
P2.q=SA[ 0.6393443 , -0.49180323,  0.49180323, -0.32786885]
α, dα_dstate = dc.proximity_gradient(P1, P2; verbose = false, pdip_tol = 1e-6)
# how to print 
println("α = $α")
println("dα_dstate = $dα_dstate")