import numpy as np

def cros(a, b):
    c = np.zeros(3)
    c[0] = a[1] * b[2] - a[2] * b[1]
    c[1] = a[2] * b[0] - a[0] * b[2]
    c[2] = a[0] * b[1] - a[1] * b[0]
    return c

def setMat(vector):
    antiMat = np.zeros((3, 3))
    antiMat[0, 1] = -vector[2]
    antiMat[0, 2] = vector[1]
    antiMat[1, 0] = vector[2]
    antiMat[1, 2] = -vector[0]
    antiMat[2, 0] = -vector[1]
    antiMat[2, 1] = vector[0]
    return antiMat

def rv2q(rv):
    q = np.zeros(4)
    n2 = np.dot(rv, rv)
    if n2 < 1.0e-8:
        q[0] = 1 - n2 * (1 / 8 - n2 / 384)
        s = 1 / 2 - n2 * (1 / 48 - n2 / 3840)
    else:
        n = np.sqrt(n2)
        n_2 = n / 2
        q[0] = np.cos(n_2)
        s = np.sin(n_2) / n
    q[1:] = s * rv
    return q