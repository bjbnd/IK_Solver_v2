import numpy as np
import sys

def unit(v):
    n = np.linalg.norm(v)
    eps = sys.float_info.epsilon
    if n < eps:
        raise Exception('RTB:unit:zero_norm', 'vector has zero norm')
    u = v/n
    return u