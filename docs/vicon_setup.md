# vicon setup

## Set up
1. Creating the vicon object. 
    - Orientation: Vehicle needs to be facing the left wall of the vicon room.
        - To do a final check: Ensure that vehicle forward direction is aligned with the x-axis (RED).
    - Position: Doesn't matter but best to put it in the middle of the vicon room setup to provide the best possible tracking

## Operation 
1. Make sure the drone starts from the origin of the room. Orientation does not matter.
2. Uses PX4 vision_pose_estimate topic


## Troubleshooting
1. Firewalls have been known to cause problems when establishing connection between the VRPN server and clients, so maybe try disabling them first if you have connection issues
2. Check that the vicon computer is connected to the same wifi network
3. 