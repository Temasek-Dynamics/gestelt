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

# Acknowledgements
1. dynamicvoronoi code from Boris Lau
