# planner_adaptor
Contains a planner adaptor which acts as an interface between the Trajectory Server and Planner. It exists as a separate standalone node. 
This allows us to treat the planner as a black box, as the planner adaptor will relay goals and executable trajectory between the planner and Trajectory Server. The purpose is to avoid having to modify the planner as much as possible to fit into the Gestelt framework.

# Design
1. Goal waypoints: Take in goal waypoints of Gestelt form, converts them to planner form and sends the goals to the planner.
2. Executable Trajectory: Takes final executable trajectory from planner, samples them and sends the sampled point to trajectory server for execution.
3. Planner heartbeat: Check timeout for planner heartbeat and sends failsafe commands to the Trajectory Server should the timeout be exceeded.
4. 2 threads are required.
