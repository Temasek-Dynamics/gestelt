# planner_adaptor
Contains a planner adaptor which acts as an interface between the Trajectory Server and Planner. It exists as a separate standalone node. 
This allows us to treat the planner as a black box, as the planner adaptor will relay goals and executable trajectory between the planner and Trajectory Server. The purpose is to avoid having to modify the planner as much as possible to fit into the Gestelt framework.

# Design
1. Goal waypoints: Take in goal waypoints of Gestelt form, converts them to planner form and sends the goals to the planner.
2. Executable Trajectory: Takes final executable trajectory from planner, samples them and sends the sampled point to trajectory server for execution.
3. Planner heartbeat: Check timeout for planner heartbeat and sends failsafe commands to the Trajectory Server should the timeout be exceeded.
4. 2 threads are required.

# Methods to implement:
Each planner will require interfacing unique to them. Hence, certain methods will need to be implemented specific to that planner.

1. `virtual void init_planner_specific_topics()`
```c++
    // Goals to be published to planner
    planner_goals_pub_ = nh.advertise<msg_pkg::msg_name>("/planner/goals", 5); 
    // Subscription to planner trajectory
    plan_traj_sub_ = nh.subscribe("/planner_adaptor/plan_trajectory", 10, &PlannerAdaptor::planTrajectoryCB, this); 
```

2. `virtual void forwardGoals(const std::vector<Eigen::Vector3d> &goal_waypoints)`
Explanation: Convert goals to a message type read by the planner and publish them using 
```c++
    // Convert goals to a message type read by the planner
    planner_goals_pub_.publish(goals);
```

3. `void planTrajectoryCB(msg_pkg::msg_name::ConstPtr& msg)`
```c++
    // Save the message into a class member for use in trajectory sampling timer callback
```

4. `virtual void samplePlanTimerCB(const ros::TimerEvent &e)`
```c++
    // Sample the saved planner trajectory 

    // Publish the executable sampled trajectory
    forwardExecTrajectory(pos, vel, acc, jerk, type_mask);
```




