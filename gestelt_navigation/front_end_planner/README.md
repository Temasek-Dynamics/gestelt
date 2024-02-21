# front_end_planner

The front end planner is a node that plans a path given a start and goal point. The planner used can be of any form (search-based or sampling-based) as long as it follows the format outlined by the front_end_planner and is implementation-specific. 

The resulting trajectory (along with any additional topics) can be fed to the back-end planner which will optimize the path for execution by the agent.

1. Subscribes to:
    1. User-specified goals.
    2. Current position of the quadrotor.
2. Publishes:
    1. A trajectory consisting of waypoints
    2. Additional topics
        1. Safe Flight Corridor
        2. Constraint 
3. Services:
    1. 