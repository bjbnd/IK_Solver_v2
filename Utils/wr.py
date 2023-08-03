import numpy as np

def anglemod(angle):
    angle = np.asarray(angle)
    wrapped_angle = np.mod(angle + np.pi, 2 * np.pi) - np.pi
    return wrapped_angle
