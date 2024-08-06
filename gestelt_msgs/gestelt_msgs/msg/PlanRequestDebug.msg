# Message used for requesting plans from planners. Primarily for debugging purposes

std_msgs/Header header

int32 agent_id          # ID of agent to plan for

geometry_msgs/Pose start # Plan start
geometry_msgs/Pose goal # Plan goal
