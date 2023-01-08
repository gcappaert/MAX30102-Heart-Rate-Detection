import numpy as np
from math import pi

def sinc(x):
    return np.sin(pi * x) /  pi * x

def sinc_filter(window, cutoff_frequency, sr):
    n = np.arange(window)
    fc = cutoff_frequency/sr
    return np.sinc(2*fc * ((n-(window-1))/2))


def blackman_window(window):
    n = np.arange(window)
    N = window
    w = 0.42 - 0.5 * np.cos(2 * np.pi * n / (N - 1)) + \
    0.08 * np.cos(4 * np.pi * n / (N - 1))
    return w

    


h = sinc_filter(115,0.5,25) * blackman_window(115)
h = h/np.sum(h)

print(h)