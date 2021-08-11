#!/usr/bin/env python3
import numpy as np


def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def angular_dist(start, end):
    start = wrap_to_pi(start)
    end = wrap_to_pi(end)
    d = end - start
    if d > np.pi:
        d -= 2 * np.pi
    elif d < -np.pi:
        d += 2 * np.pi
    return d