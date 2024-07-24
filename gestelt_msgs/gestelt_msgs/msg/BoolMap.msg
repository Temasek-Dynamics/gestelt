# 2-D Horizontal Map of Booleans
# Used for constructing generalized voronoi diagrams

std_msgs/Header header

float32 z_m   # Height of map in meters
int32 z_cm  # Height of map in centimeters

bool[] map # 1D flattened boolean array, use width and height to access as 2D Map
