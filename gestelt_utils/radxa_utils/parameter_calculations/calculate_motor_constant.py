#!/usr/bin/env python3

from math import pi
import numpy as np
import matplotlib.pyplot as plt
import csv


def main():
    kv = 4850 #RPM/
    # Air density (rho) at sea level (101 kPa), Temperature of 30 degree celcius, relative humidity of 50%
    rho = 1.152 # [kg / m3] 
    # Propeller diameter 
    prop_d = 0.0762 # [m]

    c_T = 2.9265e-04 
    c_Q = 4.7345e-09 

    motor_const_gz = (c_T * rho * prop_d**4) / (2*pi)**2

    moment_const_gz = (c_Q / c_T) * prop_d

    print("motor_const_gz: ",motor_const_gz)
    print("moment_const_gz: ",moment_const_gz)

if __name__ == '__main__':
    main()