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

    def read_csv(self, csv_filepath, num_experiments, num_rows, print_debug=False):
        """
        num_experiments: Number of experiments
        num_rows: Number of rows per experiment
        """

        self.esc_in = np.ndarray(shape=(num_experiments, num_rows), dtype=float)
        self.n_in = np.ndarray(shape=(num_experiments, num_rows), dtype=float) # [rev/min]
        self.n_in_rps = np.ndarray(shape=(num_experiments, num_rows), dtype=float) # [rev/s]
        self.T_in = np.ndarray(shape=(num_experiments, num_rows), dtype=float)
        self.P_in = np.ndarray(shape=(num_experiments, num_rows), dtype=float)

        with open(csv_filepath) as csv_file:
            csv_reader = csv.DictReader(csv_file, delimiter=',')

            arr_col = 0
            arr_row = 0

            rows = []
            for row in csv_reader:
                rows.append(row)

            for row_idx in range(len(rows)):
                row = rows[row_idx]
                if print_debug:
                    # Get column names
                    if row_idx == 0:
                        print(f'Row {row_idx} Column names are {", ".join(row)}')
                    print(f'    Row {row_idx}, Thrust: {row["Thrust (kgf)"]}, Power: {row["Electrical Power (W)"]}, RPM: {row["Motor Optical Speed (RPM)"]}, Current: {row["Current (A)"]}')

                self.esc_in[arr_col, arr_row] = float(row["ESC signal (µs)"])
                self.n_in[arr_col, arr_row] = float(row["Motor Optical Speed (RPM)"])
                self.n_in_rps[arr_col, arr_row] = float(row["Motor Optical Speed (RPM)"]) / 60 # Convert from RPM to Rev per second
                self.P_in[arr_col, arr_row] = float(row["Electrical Power (W)"])
                self.T_in[arr_col, arr_row] = float(row["Thrust (kgf)"]) * 9.80665 # Convert from kgf to N

                # Reached end of current experiment, reset row counter
                if (row_idx+1) % num_rows == 0:
                    arr_col += 1
                    arr_row = 0
                    if print_debug:
                        print(f"Experiment {arr_col}:")
                else:
                    arr_row += 1
                
            if print_debug:
                print(f'Processed {row_idx} rows within the csv file')

    def get_coefficients(self):
        self.C_T = self.get_C_T()
        self.C_P = self.get_C_P()
        self.C_Q = self.get_C_Q()
        return (self.C_T, self.C_P, self.C_Q) 

    def plot_T_against_esc(self, exp_num=0):
        """
        exp_num: Experiment number
        """
        fig, ax = plt.subplots()

        xp = np.linspace(1000, 1800, 100)

        z = np.polyfit(self.esc_in[exp_num], self.T_in[exp_num], 2)
        print(f"For Thrust against ESC Input, polynomial coeffs: {z}")
        z_poly = np.poly1d(z)

        ax.plot(self.esc_in[exp_num], self.T_in[exp_num], '.', 
                xp, z_poly(xp), '-')
        ax.set_title("Thrust against ESC Input")
        ax.grid(color = 'green', linestyle = '--', linewidth = 0.5)

        plt.show()

    def plot_T_against_rps(self, exp_num=0):
        """
        Plot thrust against revolutions per second
        exp_num: Experiment number
        """
        fig, ax = plt.subplots()

        xp = np.linspace(0, 500, 100)

        z = np.polyfit(self.n_in_rps[exp_num], self.T_in[exp_num], 2)
        print(f"For Thrust against RPS, polynomial coeffs: {z}")
        z_poly = np.poly1d(z)

        ax.plot(self.n_in_rps[exp_num], self.T_in[exp_num], '.', 
                xp, z_poly(xp), '-')
        ax.set_title("Thrust against RPS (revs / s)")
        ax.grid(color = 'green', linestyle = '--', linewidth = 0.5)

        plt.show()

    def plot_RPM_against_ESC(self, exp_num=0):
        """
        exp_num: Experiment number
        """
        fig, ax = plt.subplots()

        xp = np.linspace(1000, 1800, 100)

        z = np.polyfit(self.esc_in[exp_num], self.n_in[exp_num], 2)
        print(f"For RPM against ESC, polynomial coeffs: {z}")
        z_poly = np.poly1d(z)

        ax.plot(self.esc_in[exp_num], self.n_in[exp_num], '.', 
                xp, z_poly(xp), '-')
        ax.set_title("RPM against ESC")
        ax.grid(color = 'green', linestyle = '--', linewidth = 0.5)

        # values = np.linspace(1100, 1800, 11)
        # print(z_poly(values))

        plt.show()

    def plot_C_T(self, exp_num=0):
        fig, axs = plt.subplots(1,2)
        axs[0].plot(self.T_in[exp_num], self.C_T[exp_num])
        axs[0].set_title("C_T against Thrust")
        axs[0].grid(color = 'green', linestyle = '--', linewidth = 0.5)

        axs[1].plot(self.n_in[exp_num], self.C_T[exp_num])
        axs[1].set_title("C_T against RPM")
        axs[1].grid(color = 'green', linestyle = '--', linewidth = 0.5)

        plt.show()

    def plot_C_Q(self):
        fig, axs = plt.subplots(1,2)
        axs[0].plot(self.T_in, self.C_Q)
        axs[0].set_title("C_Q against rev/s")
        axs[0].grid(color = 'green', linestyle = '--', linewidth = 0.5)

        axs[1].plot(self.P_in, self.C_Q)
        axs[1].set_title("C_Q against Power")
        axs[1].grid(color = 'green', linestyle = '--', linewidth = 0.5)

        plt.show()

    def get_C_T(self):
        num_experiments = len(self.T_in)
        num_rows = len(self.T_in[0])
        
        C_T = np.ndarray(shape=(num_experiments, num_rows), dtype=float)

        for i in range(num_experiments): # Columns
            for j in range(num_rows): # Rows
                
                C_T[i, j] = (self.T_in[i,j] / (self.rho * (self.n_in[i,j]**2) * (self.prop_d**4)))

                if (self.C_T_max < C_T[i, j]):
                    self.C_T_max = C_T[i, j]

        return C_T

    def get_C_P(self):
        num_experiments = len(self.T_in)
        num_rows = len(self.T_in[0])

        C_P = np.ndarray(shape=(num_experiments, num_rows), dtype=float)

        for i in range(num_experiments): # Columns
            for j in range(num_rows): # Rows
                C_P[i, j] = (self.P_in[i,j] / (self.rho * (self.n_in[i,j]**3) * (self.prop_d**5)))

                if (self.C_P_max < C_P[i, j]):
                    self.C_P_max = C_P[i, j]

        return C_P

    def get_C_Q(self):
        num_experiments = len(self.T_in)
        num_rows = len(self.T_in[0])

        C_Q = np.ndarray(shape=(num_experiments, num_rows), dtype=float)

        for i in range(num_experiments): # Columns
            for j in range(num_rows): # Rows
                C_Q[i, j] = (self.C_P[i,j] / (2 * pi))

                if (self.C_Q_max < C_Q[i, j]):
                    self.C_Q_max = C_Q[i, j]

        return C_Q

    def get_motor_constant(self, C_T):   
        motor_constant = (self.C_T_max * self.rho * self.prop_d**4) / ( 2 * pi)**2
        return motor_constant

        # z = np.polyfit(self.esc_in[exp_num], self.T_in[exp_num], 2)
        # # Returns a second order polynomial, where the first term z[0] is the coefficient of the x**2
        # # In the gazebo simulation, z[0] is the motor constant where thrust = motor_constant * (rpm**2) 
        # return z[0]

    def get_moment_constant(self, C_P, C_T):   
        moment_constant = (C_P * self.prop_d) / (C_T * 2 * pi)
        # moment_constant = (self.C_Q_max * self.prop_d) / (self.C_T_max)
        return moment_constant

def main():
    # constants

    # Air density (rho) at sea level (101 kPa), Temperature of 30 degree celcius, relative humidity of 50%
    rho = 1.152 # [kg / m3] 
    # Propeller diameter 
    prop_d = 0.0762 # [m]

    # Values from https://www.getfpv.com/betafpv-1404-brushless-motor-1pc-4500kv.html
    testvalue = TestValues(rho, prop_d)

    testvalue.read_csv("thrust_data_nin_1404_4850kv/StepsTest_2023-06-27_191021.csv", 
                       num_experiments=3, num_rows = 6, print_debug=False)
    # testvalue.read_csv("thrust_data_nin_1404_4850kv/StepsTest_2023-06-28_154851.csv", 
                    #    num_experiments=2, num_rows = 11, print_debug=True)

    testvalue.get_coefficients()

    # print(f"C_P: {testvalue.C_P}")
    # print(f"C_Q: {testvalue.C_Q}")

    # Values taken from GWS 3x3 Prop: https://m-selig.ae.illinois.edu/props/volume-2/propDB-volume-2.html
    print(f"moment_constant: {testvalue.get_moment_constant(C_P = 0.143404, C_T = 0.194753)}")

    #####
    # Plots
    #####
    # testvalue.plot_T_against_esc(exp_num=0)
    testvalue.plot_T_against_rps(exp_num=0)
    # testvalue.plot_RPM_against_ESC(exp_num=1)

    # testvalue.plot_C_T()
    # testvalue.plot_C_Q()

if __name__ == '__main__':
    main()