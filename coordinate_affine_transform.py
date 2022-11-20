import math as m
try:
    from ulab import numpy as np
except ModuleNotFoundError:
    import numpy as np

def translation(xoff, yoff):
    return np.array([
        [1.0, 0.0, xoff],
        [0.0, 1.0, yoff],
        [0.0, 0.0, 1.0]
        ])

def cos_sin_deg(deg):
    """Return the cosine and sin for the given angle in degrees.

    With special-case handling of multiples of 90 for perfect right
    angles.
    """
    deg = deg % 360.0
    if deg == 90.0:
        return 0.0, 1.0
    elif deg == 180.0:
        return -1.0, 0
    elif deg == 270.0:
        return 0, -1.0
    rad = m.radians(deg)
    return m.cos(rad), m.sin(rad)

def rotation(angle, pivot=None):
    ca, sa = cos_sin_deg(angle)
    if pivot is None:
        return np.array([
            [ca, -sa, 0.0],
            [sa, ca, 0.0],
            [0.0, 0.0, 1.0]
            ])
    else:
        px, py = pivot
        return np.array([
            [ca, -sa, px - px * ca + py * sa],
            [sa, ca, py - px * sa - py * ca],
            [0.0, 0.0, 1.0]
            ])

def scale(*scaling):
    if len(scaling) == 1:
        sx = sy = float(scaling[0])
    else:
        sx, sy = scaling
    return np.array([
        [sx, 0.0, 0.0],
        [0.0, sy, 0.0],
        [0.0, 0.0, 1.0]
        ])

def find_mp(a,c):
    return tuple((c-a)/2)