# voronoi_planner
This package contains a voronoi planner

# Dependencies
```bash
sudo apt-get install libsdl-image1.2-dev 
```
# Frame transformations
Message: Goals (WORLD/Map frame)
-> 
Node: Voronoi Planner (LOCAL frame)
-> 
Message: Front End Plan (WORLD/Map frame)
->
Node: Trajectory Server (WORLD/Map frame)

# Issues
1. Fix rounding of height to multiples, right now, minimum height must be a multiple of z_separation_cm


# Acknowledgements
1. dynamicvoronoi code from Boris Lau
