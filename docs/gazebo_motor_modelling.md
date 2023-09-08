# Gazebo Motor Modelling

## Parameters

- Density
  - 1.152 kg / m3
    - Air density (rho) at sea level (101 kPa), Temperature of 30 degree celcius, relative humidity of 50%
- Propeller diameter 
  - 0.0762 m (3 inch), 76.2mm


- Thrust coefficient 
  - k_t, C_T= 2.9265e-07
- Torque coefficient 
  - k_q, C_Q= 4.7345e-09


# Maximum rotation rate [ rad/s]
rot_max = kv * V_max * efficiency * (2*pi/60) 
  - kv = 4850, 80% efficiency, V_max = 11.1V 
    - rot_max = 4850 * 11.1 * 0.60 * (2*pi/60) = 3383 rad/s

# Thrust coefficient 
## Experimental
k_t  = 2.9265e-07
## Approximated
k_t = T_max / rot_max**2

- 100% throttle at 11.1V: 
  - k_t = 0.272 / (3383**2)

- 100% throttle at 11.1V: 
  - k_t = 0.272 / (3383**2)

# Moment coefficient 
## Experimental
k_q = 4.7345e-09
## Approximated
k_q = 60 / (2*pi*kv) = 60/(2*math.pi * 4850) = 0.001968927131033757

- From (https://www.aero.psu.edu/avia/pubs/Rosenberger_MS.pdf) page 68

Ω or Max Rotational Velocity = Kv * Max Applied Voltage * Max Motor Efficiency * 2π / 60

Rotor Drag Coefficient = Thrust / (ρ * (Kv * Max Applied Voltage * Max Motor Efficiency / 60) ² * Propeller diameter ⁴)

Rolling Moment Coefficient = Using SST turbulence model from reference, and bent/round wings/propellers: ~0.0220 for angle of attack smaller than 16 degrees 
-> I'm keeping this defaulted to 1E-06 for now.

From (https://github.com/engcang/mavros-gazebo-application/tree/master/px4_model_making#-motor)


## Masses
*Base includes motors, range sensor, radxa SBC and FCU.

Base (W/ battery): 123 g
Base (W/ battery): 195g

Base (W/ Battery and Vilota): 302g

radxa: 12g
Battery: 70g
Motors: 10g
Propellers: 1.5g
Flight Controller Unit: 12.5g

Main frame: 21.5g
Battery holder: 6.5g
Radxa holder: 5g

## Calculating Mass Moment of inertia 


### References 
1. [How to Find Mass Moment of Inertia | Mechanics Statics | (Solved Examples)](https://www.youtube.com/watch?v=Bls5KnQOWkY)
2.  
