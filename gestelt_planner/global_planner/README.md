# global_planner
Contains a global planner for creating global plans to be optimized further.

# Notes
- rename gridnode to avoid conflict with another predefined definition

- implement timeout

- Convert plannerBase to a parent class and have astar class inherit from it

- Add tiebreaker

- Planner plans within the UAV's frame
- Incoming goals need to be converted to the UAV's frame
