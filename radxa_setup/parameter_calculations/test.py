#!/usr/bin/env python3

from math import pi
import numpy as np
import matplotlib.pyplot as plt
import csv


def main():
    x = np.array([0.0, 1.0, 2.0, 3.0,  4.0,  5.0])
    y = np.array([0.0, 0.8, 0.9, 0.1, -0.8, -1.0])

    xp = np.linspace(-2, 6, 50)

    z_2 = np.polyfit(x, y, 2)
    print(f"z_2: {z_2}")
    z_2_y = [z_2[0] * val**2 + z_2[1] * val + z_2[2] for val in xp] 

    z_3 = np.polyfit(x, y, 3)
    print(f"z_3: {z_3}")
    z_3_poly = np.poly1d(z_3)

    z_30 = np.polyfit(x, y, 30)
    print(f"z_30: {z_30}")
    z_30_poly = np.poly1d(z_30)


    fig, ax = plt.subplots()
    ax.plot(x, y, '.', xp, z_3_poly(xp), '-', xp, z_30_poly(xp), '--')
    plt.ylim(-2,2)
    plt.show() 

if __name__ == '__main__':
    main()