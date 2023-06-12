# global_planner
Contains a global planner for creating global plans to be optimized further.

# Notes
- ZJU created an internal map data structure so as to store data about existing nodes it has searched. What is one way we can do the same without a memory expensive data structure?
  - Soln A
    - Keep a hashset of the parents 
    - Use a hashset to store the states of the explored nodes

- Planner plans within the UAV's frame
- Incoming goals need to be converted to the UAV's frame
