# grid_map
The grid_map package implements a 3d voxel based occupancy grid utilizing [Bonxai](https://github.com/facontidavide/Bonxai) to do occupancy checking.

## Local and global map
grid_map keeps both a local_map and a global_map. The local_map is contained within a sliding 3D bounded box centered on the UAV's position, which is then utilised for planning.

The global_map is the probabilistic map constructed over time as the UAV receives new sensor information.