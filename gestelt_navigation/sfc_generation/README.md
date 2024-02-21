# sfc_generation
Safe flight corridor generation

# Improvements over original algorithm
1. Add minimum candidate sphere volume and minimum volume of intersection between spheres. 
2. 

# Proposed improvements 
1. Adaptive weights
2. Adapt to changes in sampling distribution when batch sampling cannot be carried out 
2. Different sampling distribution

# Issues
1. Astar assumes a point mass and does not take into account geometrical volume of robot. Paths found through a star might not be feasible unless map is inflated. 
