# A stack of boolean maps

float32 z_separation_m      # z separation of horizontal planes [m]
int32 z_separation_cm       # z separation of horizontal planes [cm]

float32 max_height_m        # Maximum height [m]
int32 max_height_cm         # Maximum height [cm]

geometry_msgs/Point origin  # Map origin [m]

float32 width               # Width of map [m]
float32 height              # Height of map [m]
float32 resolution          # Resultion of map [m]

gestelt_msgs/BoolMap[] bool_maps # Array of boolean maps
