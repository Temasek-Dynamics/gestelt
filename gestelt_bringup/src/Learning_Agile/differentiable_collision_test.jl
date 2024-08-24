import Pkg
Pkg.activate(".")
# Pkg.add("StaticArrays")
# Pkg.add("MeshCat")
Pkg.add(path="/home/tlab-uav/DifferentiableCollisions.jl")
import DifferentiableCollisions as dc
import MeshCat as mc
using StaticArrays
using LinearAlgebra

vis = mc.Visualizer()
mc.open(vis)

cone = dc.Cone(3.0, deg2rad(22))
cone.r = @SVector randn(3)
cone.q = normalize((@SVector randn(4)))

# build primitive scaled by α = 1.0
dc.build_primitive!(vis, cone, :cone; α = 1.0, color = mc.RGBA(1,0,0,1.0))

# update position and attitude
dc.update_pose!(vis[:cone], cone)