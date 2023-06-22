#!/usr/bin/env python3

from math import pi
import numpy as np
import matplotlib.pyplot as plt

class TestValues:
    def __init__(self, rho, prop_d):
        self.rho = rho
        self.prop_d = prop_d
        # Datapoints
        self.n_in = [] # [s**-1]
        self.P_in = [] # [W]
        self.T_in = [] # [N]

        # Coefficients
        self.C_T = []
        self.C_P = []
        self.C_Q = []

        self.C_T_max = 0
        self.C_P_max = 0
        self.C_Q_max = 0

    def add_datapoint(self, n_in, P_in, T_in):
        self.n_in.append(n_in)
        self.P_in.append(P_in)
        self.T_in.append(T_in)

    def get_coefficients(self):
        self.C_T = self.get_C_T()
        self.C_P = self.get_C_P()
        self.C_Q = self.get_C_Q()
        return (self.C_T, self.C_P, self.C_Q) 

    def plot_T_against_n(self):
        fig, ax = plt.subplots()

        z = np.polyfit(self.n_in, self.T_in, 2)
        print(f"z: {z}")

        ax.plot(self.n_in, self.T_in)
        ax.set_title("Thrust against n")
        # ax.set_y_label("Thrust [N]")
        # ax.set_x_label("n [Rev/s]")
        ax.grid(color = 'green', linestyle = '--', linewidth = 0.5)

        plt.show()


    def plot_C_T(self):
        fig, axs = plt.subplots(1,2)
        axs[0].plot(self.T_in, self.C_T)
        axs[0].set_title("C_T against rev/s")
        axs[0].grid(color = 'green', linestyle = '--', linewidth = 0.5)

        # axs[1].plot(self.P_in, self.C_T)
        # axs[1].set_title("C_T against Power")
        # axs[1].grid(color = 'green', linestyle = '--', linewidth = 0.5)

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
        C_T = []
        for i in range(len(self.T_in)):
            C_T.append(self.T_in[i] / (self.rho * (self.n_in[i]**2) * (self.prop_d**4)))

            if (self.C_T_max < C_T[i]):
                self.C_T_max = C_T[i]

        return C_T

    def get_C_P(self):
        C_P = []
        for i in range(len(self.P_in)):
            C_P.append(self.P_in[i] / (self.rho * (self.n_in[i]**3) * (self.prop_d**5)))

            if (self.C_P_max < C_P[i]):
                self.C_P_max = C_P[i]

        return C_P

    def get_C_Q(self):
        C_Q = []
        for i in range(len(self.C_P)):
            C_Q.append(self.C_P[i] / (2 * pi))

            if (self.C_Q_max < C_Q[i]):
                self.C_Q_max = C_Q[i]

        return C_Q

    def get_motor_constant(self):   
        motor_constant = (self.C_T_max * self.rho * self.prop_d**4) / ( 2 * pi)**2
        return motor_constant

    def get_moment_constant(self):   
        moment_constant = (self.C_P_max * self.prop_d) / (self.C_T_max * 2 * pi)
        return moment_constant

def main():
    # constants

    # Air density (rho) at sea level (101 kPa), Temperature of 30 degree celcius, relative humidity of 50%
    rho = 1.152 # [kg / m3] 
    # Propeller diameter 
    prop_d = 0.0762 # [m]

    # Values from https://www.getfpv.com/betafpv-1404-brushless-motor-1pc-4500kv.html
    testvalue = TestValues(rho, prop_d)
    testvalue.add_datapoint((1/60) * 17493, 26.27, 0.06)
    testvalue.add_datapoint((1/60) * 19756, 36.44, 0.08)
    testvalue.add_datapoint((1/60) * 22020, 46.61, 0.1)
    testvalue.add_datapoint((1/60) * 24283, 56.78, 0.12)
    testvalue.add_datapoint((1/60) * 26186, 68.06, 0.14)
    testvalue.add_datapoint((1/60) * 27549, 81.01, 0.16)
    testvalue.add_datapoint((1/60) * 29010, 93.69, 0.18)
    testvalue.add_datapoint((1/60) * 30766, 105.58, 0.2)
    testvalue.add_datapoint((1/60) * 32413, 118.85, 0.22)
    testvalue.add_datapoint((1/60) * 33448, 139.99, 0.24)
    testvalue.add_datapoint((1/60) * 34482, 161.12, 0.26)
    testvalue.add_datapoint((1/60) * 36100, 188.2, 0.28)
    testvalue.add_datapoint((1/60) * 45454, 286.2, 0.411)

    testvalue.get_coefficients()

    print(f"Max C_T: {testvalue.C_T_max}")
    print(f"Motor constant: {testvalue.get_motor_constant()}")
    print(f"Moment constant: {testvalue.get_moment_constant()}")

    #####
    # Plots
    #####
    testvalue.plot_T_against_n()

    # testvalue.plot_C_T()
    # testvalue.plot_C_Q()


if __name__ == '__main__':
    main()