# 2-D Horizontal Map of Booleans
# Used for constructing generalized voronoi diagrams

std_msgs/Header header

geometry_msgs/Point origin # Map origin

float32 width # Width of map
float32 height # Height of map

bool[] map # 1D flattened boolean array, use width and height to access as 2D Map
