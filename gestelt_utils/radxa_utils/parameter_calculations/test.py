#!/usr/bin/env python3

from math import pi
import numpy as np
import matplotlib.pyplot as plt

def main():

    x = np.array([1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600])
    y = np.array([6690, 10316, 13510, 16476, 19112, 21518, 23676, 25601, 27317, 28892])

    xp = np.linspace(1000, 1800, 100)

    z = np.polyfit(x, y, 2)
    z_poly = np.poly1d(z)

    print(f"Points of interest on polynomial: ({1650}, {z_poly(1650)}) and ({1700}, {z_poly(1750)})")

    fig, ax = plt.subplots()
    ax.plot(x, y, '.', xp, z_poly(xp), '-')
    plt.show() 

if __name__ == '__main__':
    main()