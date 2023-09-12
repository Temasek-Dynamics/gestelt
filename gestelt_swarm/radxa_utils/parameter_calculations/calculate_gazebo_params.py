#!/usr/bin/env python3

from math import pi
import numpy as np
import matplotlib.pyplot as plt
import csv

class TestValues:
    def __init__(self, rho, prop_d):
        self.rho = rho
        self.prop_d = prop_d

        self.C_T_max = 0
        self.C_P_max = 0
        self.C_Q_max = 0

    def read_csv(self, csv_filepath, thrust_in_grams=False, print_debug=False):

        with open(csv_filepath) as csv_file:
            csv_reader = csv.DictReader(csv_file, delimiter=',')

            rows = []
            for row in csv_reader:
                rows.append(row)
            
            data_in_shape = (1, len(rows)-1)

            self.esc_in = np.ndarray(shape=data_in_shape, dtype=float) # ESC Signal [microsecond]
            self.torque_in = np.ndarray(shape=data_in_shape, dtype=float) # Torque [N * meters]
            self.thrust_in = np.ndarray(shape=data_in_shape, dtype=float) # Thrust [Kg f]
            self.thrust_in_newtons = np.ndarray(shape=data_in_shape, dtype=float) # Thrust [N]
            self.n_in = np.ndarray(shape=data_in_shape, dtype=float) # Motor speed [rev/min]
            self.n_in_radians = np.ndarray(shape=data_in_shape, dtype=float) # Motor speed [rad/s]
            self.P_in = np.ndarray(shape=data_in_shape, dtype=float) # Electrical Power [W] 

            print(f'Column names are {", ".join(rows[0])}')

            for row_idx in np.arange(1,len(rows)):
                row = rows[row_idx]

                # print(f'    Row {row_idx}, Thrust: {row["Thrust (kgf)"]}, Power: {row["Electrical Power (W)"]}, RPM: {row["Motor Optical Speed (RPM)"]}, Current: {row["Current (A)"]}')
                self.esc_in[0, row_idx - 1] = float(row["ESC signal (µs)"])
                self.torque_in[0, row_idx - 1] = -float(row["Torque (N·m)"])

                self.thrust_in[0, row_idx - 1] = float(row["Thrust (kgf)"])
                self.thrust_in_newtons[0, row_idx - 1] = float(row["Thrust (kgf)"]) * 9.80665 # Convert from kgf to N

                self.n_in[0, row_idx - 1] = float(row["Motor Optical Speed (RPM)"])
                self.n_in_radians[0, row_idx - 1] = float(row["Motor Optical Speed (RPM)"]) * (2*pi / 60 ) # Convert from RPM to Radians/second
                self.P_in[0, row_idx - 1] = float(row["Electrical Power (W)"])

            if print_debug:
                print(f'Processed {row_idx} rows within the csv file')

    def plotThrust_AngVel(self):
        """
        Plot thrust against revolutions per second
        """
        fig, ax = plt.subplots()
        ax.grid(color = 'green', linestyle = '--', linewidth = 0.5)
        # ax.set_title("Thrust (N) against Angular velocity (Rad/s)")
        ax.set_xlabel("Angular velocity (Rad/s)", fontweight= 'bold')
        ax.set_ylabel("Thrust (N)", fontweight= 'bold')

        xp = np.linspace(750, 3250, 100)
        
        # Fit 2nd order polynomial
        z = np.polyfit(self.n_in_radians[0], self.thrust_in_newtons[0], 2)
        print(f"For Thrust (N) against Angular velocity (Rad/s), polynomial coeffs: {z}")
        z_poly = np.poly1d(z)

        ax.plot(self.n_in_radians[0], self.thrust_in_newtons[0], '.', 
                xp, z_poly(xp), '-')

        # ax.plot(self.n_in_radians, self.thrust_in_newtons, '.')

        plt.show()

    def plotTorque_AngVel(self):
        """
        Plot thrust against revolutions per second
        """
        fig, ax = plt.subplots()
        ax.grid(color = 'green', linestyle = '--', linewidth = 0.5)
        # ax.set_title("Torque (N.m) against Angular velocity (Rad/s)")
        ax.set_xlabel("Angular velocity (Rad/s)", fontweight= 'bold')
        ax.set_ylabel("Torque (N.m)", fontweight= 'bold')

        xp = np.linspace(750, 3250, 100)

        # Fit 2nd order polynomial
        z = np.polyfit(self.n_in_radians[0], self.torque_in[0], 2)
        print(f"For Torque (N.m) against Angular velocity (Rad/s), polynomial coeffs: {z}")
        z_poly = np.poly1d(z)

        ax.plot(self.n_in_radians[0], self.torque_in[0], '.', 
                xp, z_poly(xp), '-')

        plt.show()

    # def plot_T_against_esc(self, exp_num=0):
    #     """
    #     exp_num: Experiment number
    #     """
    #     fig, ax = plt.subplots()

    #     xp = np.linspace(1000, 1800, 100)

    #     z = np.polyfit(self.esc_in[exp_num], self.thrust_in[exp_num], 2)
    #     print(f"For Thrust against ESC Input, polynomial coeffs: {z}")
    #     z_poly = np.poly1d(z)

    #     ax.plot(self.esc_in[exp_num], self.thrust_in[exp_num], '.', 
    #             xp, z_poly(xp), '-')
    #     ax.set_title("Thrust against ESC Input")
    #     ax.grid(color = 'green', linestyle = '--', linewidth = 0.5)

    #     plt.show()

    # def plot_n_against_ESC(self, exp_num=0):
    #     """
    #     exp_num: Experiment number
    #     """
    #     fig, ax = plt.subplots()

    #     xp = np.linspace(1000, 1800, 100)

    #     z = np.polyfit(self.esc_in[exp_num], self.n_in[exp_num], 2)
    #     print(f"For n against ESC, polynomial coeffs: {z}")
    #     z_poly = np.poly1d(z)

    #     ax.plot(self.esc_in[exp_num], self.n_in[exp_num], '.', 
    #             xp, z_poly(xp), '-')
    #     ax.set_title("n against ESC")
    #     ax.grid(color = 'green', linestyle = '--', linewidth = 0.5)

    #     # values = np.linspace(1100, 1800, 11)
    #     # print(z_poly(values)) 

    #     plt.show()


    # def get_coefficients(self):
    #     self.C_T = self.get_C_T()
    #     self.C_P = self.get_C_P()
    #     self.C_Q = self.get_C_Q()
    #     return (self.C_T, self.C_P, self.C_Q) 

    # def plot_C_T(self, exp_num=0):
    #     fig, axs = plt.subplots(1,2)
    #     axs[0].plot(self.thrust_in[exp_num], self.C_T[exp_num])
    #     axs[0].set_title("C_T against Thrust")
    #     axs[0].grid(color = 'green', linestyle = '--', linewidth = 0.5)

    #     axs[1].plot(self.n_in[exp_num], self.C_T[exp_num])
    #     axs[1].set_title("C_T against RPM")
    #     axs[1].grid(color = 'green', linestyle = '--', linewidth = 0.5)

    #     plt.show()

    # def plot_C_Q(self):
    #     fig, axs = plt.subplots(1,2)
    #     axs[0].plot(self.thrust_in, self.C_Q)
    #     axs[0].set_title("C_Q against rev/s")
    #     axs[0].grid(color = 'green', linestyle = '--', linewidth = 0.5)

    #     axs[1].plot(self.P_in, self.C_Q)
    #     axs[1].set_title("C_Q against Power")
    #     axs[1].grid(color = 'green', linestyle = '--', linewidth = 0.5)

    #     plt.show()

    # def get_C_T(self):
    #     num_exp = len(self.thrust_in)
    #     num_rows = len(self.thrust_in[0])
        
    #     C_T = np.ndarray(shape=(num_exp, num_rows), dtype=float)

    #     for i in range(num_exp): # Columns
    #         for j in range(num_rows): # Rows
                
    #             C_T[i, j] = (self.thrust_in[i,j] / (self.rho * (self.n_in[i,j]**2) * (self.prop_d**4)))

    #             if (self.C_T_max < C_T[i, j]):
    #                 self.C_T_max = C_T[i, j]

    #     return C_T

    # def get_C_P(self):
    #     num_exp = len(self.thrust_in)
    #     num_rows = len(self.thrust_in[0])

    #     C_P = np.ndarray(shape=(num_exp, num_rows), dtype=float)

    #     for i in range(num_exp): # Columns
    #         for j in range(num_rows): # Rows
    #             C_P[i, j] = (self.P_in[i,j] / (self.rho * (self.n_in[i,j]**3) * (self.prop_d**5)))

    #             if (self.C_P_max < C_P[i, j]):
    #                 self.C_P_max = C_P[i, j]

    #     return C_P

    # def get_C_Q(self):
    #     num_exp = len(self.thrust_in)
    #     num_rows = len(self.thrust_in[0])

    #     C_Q = np.ndarray(shape=(num_exp, num_rows), dtype=float)

    #     for i in range(num_exp): # Columns
    #         for j in range(num_rows): # Rows
    #             C_Q[i, j] = (self.C_P[i,j] / (2 * pi))

    #             if (self.C_Q_max < C_Q[i, j]):
    #                 self.C_Q_max = C_Q[i, j]

    #     return C_Q

    # def get_motor_constant(self, C_T):   
    #     motor_constant = (self.C_T_max * self.rho * self.prop_d**4) / ( 2 * pi)**2
    #     return motor_constant

    #     # z = np.polyfit(self.esc_in[exp_num], self.thrust_in[exp_num], 2)
    #     # # Returns a second order polynomial, where the first term z[0] is the coefficient of the x**2
    #     # # In the gazebo simulation, z[0] is the motor constant where thrust = motor_constant * (rpm**2) 
    #     # return z[0]

    # def get_moment_constant(self, C_P, C_T):   
    #     moment_constant = (C_P * self.prop_d) / (C_T * 2 * pi)
    #     # moment_constant = (self.C_Q_max * self.prop_d) / (self.C_T_max)
    #     return moment_constant

def main():
    # constants

    # Air density (rho) at sea level (101 kPa), Temperature of 30 degree celcius, relative humidity of 50%
    rho = 1.152 # [kg / m3] 
    # Propeller diameter 
    prop_d = 0.0762 # [m]

    # Values from https://www.getfpv.com/betafpv-1404-brushless-motor-1pc-4500kv.html
    testvalue = TestValues(rho, prop_d)

    testvalue.read_csv("thrust_data/nuswarm_9_12_consolidated_clean.csv", thrust_in_grams=False, print_debug=False)
    #####
    # Plots
    #####
    # testvalue.plotThrust_AngVel()
    testvalue.plotTorque_AngVel()

if __name__ == '__main__':
    main()