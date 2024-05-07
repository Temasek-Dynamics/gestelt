# Scenarios
A few scenarios are provided to test the planner algorithms:
- forest_single
- antipodal_swap
- narrow_corridor

See [scenarios](../gestelt_bringup/launch/scenarios/scenarios/)

# All scenarios
```bash
# empty_map scenario
./scenario.sh -s empty_map
# forest_single scenario
./scenario.sh -s forest_single
# narrow_corridor
# ./scenario.sh -s narrow_corridor
# antipodal_swap
./scenario.sh -s antipodal_swap8_empty
./scenario.sh -s antipodal_swap10_empty
./scenario.sh -s antipodal_swap10
./scenario.sh -s antipodal_swap12
# Forest scene for 16 drones 
./scenario.sh -s forest16_50obs
./scenario.sh -s forest10_50obs
./scenario.sh -s forest8_50obs
```

## Generate a new map 
```bash
# Edit gestelt_swarm/fake_map/config/forest_generate.yaml
roslaunch fake_map fake_map_generator.launch
``` 
