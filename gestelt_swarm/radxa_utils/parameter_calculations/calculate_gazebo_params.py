#!/usr/bin/env python3

import math
from math import pi
import numpy as np
import matplotlib.pyplot as plt
import csv

class TestValues:
    def __init__(self):
        pass

    def read_csv(self, csv_filepath):

        with open(csv_filepath) as csv_file:
            csv_reader = csv.DictReader(csv_file, delimiter=',')

            rows = []
            for row in csv_reader:
                rows.append(row)
            
            data_in_shape = (1, len(rows)-1)
            
            self.data_in = {
                "ESC signal (µs)": np.ndarray(shape=data_in_shape, dtype=float),
                "Torque (N·m)": np.ndarray(shape=data_in_shape, dtype=float),
                "Thrust (kgf)": np.ndarray(shape=data_in_shape, dtype=float),
                "Motor Optical Speed (RPM)": np.ndarray(shape=data_in_shape, dtype=float),
                "Electrical Power (W)": np.ndarray(shape=data_in_shape, dtype=float),

                "Current (A)": np.ndarray(shape=data_in_shape, dtype=float),

                "Thrust (N)": np.ndarray(shape=data_in_shape, dtype=float),
                "Motor Optical Speed (rad/s)": np.ndarray(shape=data_in_shape, dtype=float),
                "Motor Efficiency": np.ndarray(shape=data_in_shape, dtype=float),
            }

            # print(f'Column names are {", ".join(rows[0])}')

            for row_idx in np.arange(1,len(rows)):
                row = rows[row_idx]

                # print(f'    Row {row_idx}, Thrust: {row["Thrust (kgf)"]}, Power: {row["Electrical Power (W)"]}, RPM: {row["Motor Optical Speed (RPM)"]}, Current: {row["Current (A)"]}')
                self.data_in["ESC signal (µs)"][0, row_idx - 1] = float(row["ESC signal (µs)"])
                self.data_in["Torque (N·m)"][0, row_idx - 1] = float(row["Torque (N·m)"])

                self.data_in["Thrust (kgf)"][0, row_idx - 1] = float(row["Thrust (kgf)"])

                self.data_in["Motor Optical Speed (RPM)"][0, row_idx - 1] = float(row["Motor Optical Speed (RPM)"])
                self.data_in["Electrical Power (W)"][0, row_idx - 1] = float(row["Electrical Power (W)"])

                self.data_in["Current (A)"][0, row_idx - 1] = float(row["Current (A)"])

            # Derived values
            self.data_in["Thrust (N)"] = self.data_in["Thrust (kgf)"] * 9.80665 # Convert from kgf to N
            self.data_in["Motor Optical Speed (rad/s)"] = self.data_in["Motor Optical Speed (RPM)"] * (2*pi / 60 ) # Convert from RPM to Radians/second
            # Efficiency = (torque * angular velocity) / electrical power
            self.data_in["Motor Efficiency"] = np.multiply(self.data_in["Torque (N·m)"], self.data_in["Motor Optical Speed (rad/s)"]) / self.data_in["Electrical Power (W)"]

    def plotGraph(self, ax, x_key, y_key, poly_fit_order=2, x_lim = [500, 3100]):
        """Plot a graph 

        Args:
            x_key (_type_): _description_
            y_key (_type_): _description_
            poly_fit_order (int, optional): _description_. Defaults to 2.
            x_lim (list, optional): _description_. Defaults to [500, 3100].

        Returns:
            Coefficient (float): Coefficient of the highest order term
        """
        x = self.data_in[x_key]
        y = self.data_in[y_key]
        
        # Fit 2nd order polynomial
        z = np.polyfit(x[0], y[0], poly_fit_order)
        print(f"For {y_key} against {x_key}, polynomial coeffs: {z}")
        z_poly = np.poly1d(z)
        
        ax.set_title(f"{y_key} against {x_key}")
        ax.grid(color = 'green', linestyle = '--', linewidth = 0.5)
        ax.set_xlabel(x_key, fontweight= 'bold')
        ax.set_ylabel(y_key, fontweight= 'bold')
        xp = np.linspace(x_lim[0], x_lim[1], 100)
        ax.plot(x, y, '.b', 
                xp, z_poly(xp), '-r')

        return z[0]

def main():
    # constants

    # Air density (rho) at sea level (101 kPa), Temperature of 30 degree celcius, relative humidity of 50%
    rho = 1.152 # [kg / m3] 
    # Propeller diameter 
    prop_d = 0.0762 # [m]

    # Values from https://www.getfpv.com/betafpv-1404-brushless-motor-1pc-4500kv.html
    testvalue = TestValues()

    # testvalue.read_csv("thrust_data/18_9_23_flywoo_nin1404_bench_test_cleaned.csv")
    testvalue.read_csv("thrust_data/18_9_23_flywoo_nin1404_bench_test_cleaned_wo_top_vals.csv")

    #####
    # Plots
    #####

    fig, axs = plt.subplots(2,2, figsize=(15, 15))

    C_T = testvalue.plotGraph(axs[0,0], x_key="Motor Optical Speed (rad/s)", y_key = "Thrust (N)", poly_fit_order=2, x_lim=[500,3500])
    C_M = testvalue.plotGraph(axs[0,1], x_key="Motor Optical Speed (rad/s)", y_key = "Torque (N·m)", poly_fit_order=2, x_lim=[500,3500])
    testvalue.plotGraph(axs[1,0], x_key="Motor Optical Speed (rad/s)", y_key = "Motor Efficiency", poly_fit_order=2, x_lim=[500,3500])
    testvalue.plotGraph(axs[1,1], x_key="Motor Optical Speed (rad/s)", y_key = "Current (A)", poly_fit_order=3, x_lim=[500,3500])


    #####
    # 
    #####
    # Max thrust [N] (from datasheet)
    T_max = 0.3224 * 9.81
    # T_max = 2.8
    # Max angular velocity [rad/s] (From calculation)
    # ang_vel_max = 3000
    ang_vel_max = math.sqrt(T_max / C_T)
    # Max thrust [N.m] (From calculation)
    torque_max = (ang_vel_max**2) * C_M

    # # Motor constant based on max thrust from manufacturer datasheet 
    # gz_motor_constant_max_T = T_max / (ang_vel_max**2)
    # gz_moment_constant_max_T = torque_max/T_max

    # Motor constant based on experiments
    gz_motor_constant_actual = C_T
    gz_moment_constant_actual = C_M/C_T

    print(f"C_T:{C_T}, C_M:{C_M}")
    print("T_max [N]: ", T_max)
    print("ang_vel_max [rad/s]: ", ang_vel_max)
    print("Torque_max [N.m]: ", torque_max)

    # print(f"From manufacturer datasheet: motor_constant={gz_motor_constant_max_T}, moment_constant={gz_moment_constant_max_T}")
    print(f"From actual data: motor_constant={gz_motor_constant_actual}, moment_constant={gz_moment_constant_actual}")

    plt.show()

if __name__ == '__main__':
    main()