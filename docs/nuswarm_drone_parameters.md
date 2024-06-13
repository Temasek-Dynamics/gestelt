# Physical dimensions
Volume: 0.178m x 0.178m x 0.075m.
Mass: 0.2 kg (without camera), 0.35 kg (with camera) 
Assuming it is a spherical object:
- Diameter: 0.252m
- Radius: 0.126m

## Recommended inflation for obstacle avoidance and planning
- At least 0.126m
- Recommended inflation: 0.126m + tracking_error

# Dynamical constraints
Can produce up to 8 Newtons of thrust in z axis.
Assuming 6 newtons of thrust:
- Assuming without mounted camera 0.2 kg of mass:
    - max_acc = 30.0 m/s^2
- Assuming with mounted camera at 0.35 kg of mass:
    - max_acc = 20.0 m/s^2