# vicon setup

## Set up
1. Creating the vicon object. 
    - Orientation: Vehicle needs to be facing the left wall of the vicon room.
        - To do a final check: Ensure that vehicle forward direction is aligned with the x-axis (RED).
    - Position: Doesn't matter but best to put it in the middle of the vicon room setup to provide the best possible tracking

## Operation 
1. Make sure the drone starts from the origin of the room. Orientation does not matter.
2. Uses PX4 vision_pose_estimate topic

# Checks
- Frame transformations
    - Vicon provides pose in ENU/FLU
    - Need to change "odom" plugin params? Transform from map frame to base_link_frd?

- Consider sending odometry to "/mavros/odometry/out", using topics from /pose and /twist

