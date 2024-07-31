std_msgs/Header header
# UInt32 drone_id                         # drone_id to assign goal to       
int32 task_id                           # ID of task 

# Vector of goals
geometry_msgs/Transform[] transforms    # goal positions
geometry_msgs/Twist[] velocities        # goal velocities
geometry_msgs/Twist[] accelerations     # goal accelerations