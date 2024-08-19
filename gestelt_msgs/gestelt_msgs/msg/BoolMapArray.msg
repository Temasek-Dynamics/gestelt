# A stack of boolean maps

float64 z_separation_m      # z separation of horizontal planes [m]
int32 z_separation_cm       # z separation of horizontal planes [cm]

float64 min_height_m        # Minimum height [m]
int32 min_height_cm         # Minimum height [cm]

float64 max_height_m        # Maximum height [m]
int32 max_height_cm         # Maximum height [cm]

geometry_msgs/Point origin  # Map origin [m]

float64 width               # Width of map [m]
float64 height              # Height of map [m]
float64 resolution          # Resultion of map [m]

gestelt_msgs/BoolMap[] bool_maps # Array of boolean maps
