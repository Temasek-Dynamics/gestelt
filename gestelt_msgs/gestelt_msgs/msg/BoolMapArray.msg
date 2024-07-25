# A stack of boolean maps

float32 z_separation_m      # z separation of horizontal planes in meters
int32 z_separation_cm       # z separation of horizontal planes in centi-meters

float32 max_height_m        # Maximum height in meters
int32 max_height_cm         # Maximum height in centimeters

geometry_msgs/Point origin  # Map origin

float32 width               # Width of map
float32 height              # Height of map

gestelt_msgs/BoolMap[] bool_maps 
