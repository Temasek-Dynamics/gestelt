```bash
# Build in release mode
catkin build --env-cache
```

# Add packages to skiplist to save time
```bash
catkin config --skiplist bonxai catkin_simple ikd_tree planner_adaptor logger gestelt_test trajectory_inspector fake_drone fake_map swarm_collision_checker central_benchmark  jps3d trajectory_server gestelt_debug_msgs gestelt_msgs vrpn_client_ros simple_quad_sim path_searching decomp_test_node

# convex_decomp_util decomp_ros_msgs decomp_ros_utils decomp_util grid_map traj_opt traj_utils global_planner navigator 

# Clear from skiplist
catkin config --no-skiplist
```

# Optimization flags
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
# Release with optimization and debug symbols and 
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo 

# No optimization done
catkin_make -DCMAKE_CXX_FLAGS=-O0
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

