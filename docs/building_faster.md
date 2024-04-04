```bash
# Build in release mode
catkin build --env-cache
```

# Add packages to skiplist to save time
```bash
catkin config --skiplist bonxai catkin_simple ikd_tree planner_adaptor logger gestelt_test trajectory_inspector fake_drone fake_map swarm_bridge swarm_collision_checker central_benchmark ego_planner_fsm traj_utils traj_opt jps3d trajectory_server gestelt_debug_msgs gestelt_msgs vrpn_client_ros simple_quad_sim path_searching


# convex_decomp_util decomp_ros_msgs decomp_ros_utils decomp_test_node decomp_util grid_map

# Clear from skiplist
catkin config --no-skiplist
```

# Optimization flags
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```
