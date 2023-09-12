#!/usr/bin/env python3

from math import pi
import numpy as np
import matplotlib.pyplot as plt

def main():

    data_in_shape = (5, 1)
    esc_in = np.ndarray(shape=data_in_shape, dtype=float) # ESC Signal [microsecond]
    print(esc_in[0,1])

if __name__ == '__main__':
    main()