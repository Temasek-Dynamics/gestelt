# fake_map
This package has 2 executables. 
- `fake_map` simulates a mounted sensor on the drone. This is done by taking in the transformation of the drone (via `/tf`) relative to the world frame and outputting a point cloud of the map.
- `fake_map_publisher` simply publishes the global fake map (originally a PCD file) for visualization
There is also a `fake_laser` library used by `fake_map` which simulates a 3d lidar.    

# Acknowledgements
The point cloud sensor simulator (named as `fake_laser`) is taken from ZheJiang university's code for simulating drones