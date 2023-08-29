# vicon setup
This document highlights some important pointers for using Vicon together with a PX4 drone. 

## Vicon room dimensions
3.5 (width) * 4.64m (Length) 

## Set up
1. Creating the vicon object. 
    - Orientation: Vehicle's x-axis (forward) needs to be aligned with the x-axis (RED) of the vicon system.
    - Position: Doesn't matter but best to put it in the middle of the vicon room setup to provide the best possible angle and position for initializing the vicon objects.

## Important pointers 
1. PX4-Mavros takes the current starting position as the origin frame (0,0,0). For your own testing purposes, make sure you understand this. It is advised to keep the starting position consistent. 
2. Uses PX4 `vision_pose_estimate` topic
3. PX4 noise param
4. PX4 delay param

## Troubleshooting
1. Firewalls have been known to cause problems when establishing connection between the VRPN server and clients, so maybe try disabling them first if you have connection issues
2. Check that the vicon computer is connected to the same wifi network
3. Vicon cameras not starting up:
    - Is computer connected to Vicon LAN? If not, on the Vicon computer, try disabling and re-enabling the network adapter for the vicon LAN.