# 2-D Horizontal Map of Booleans
# Used for constructing generalized voronoi diagrams

std_msgs/Header header

float64 z_m     # [m] Height of map 
int32 z_cm      # [cm] Height of map 

bool[] map # 1D flattened boolean array, use width and height to access as 2D Map
