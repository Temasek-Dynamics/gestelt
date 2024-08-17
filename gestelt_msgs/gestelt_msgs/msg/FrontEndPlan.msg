# Space time path

std_msgs/Header header  
int32 agent_id          # ID of agent 

geometry_msgs/Pose[] plan  # Vector of poses representing the plan
float64 plan_start_time     # [s] The plan starting time
int32[] plan_time         # [space-time units] Time vector for each pose on the plan
