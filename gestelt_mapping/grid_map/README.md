# grid_map
The grid_map package implements a 3d voxel based occupancy grid utilizing octree to do occupancy checking.
As 3d mapping is computationally intensive, grid_map only keeps maps within a given local bound around the drone and discards anything outside the bound.