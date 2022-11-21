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

def test():

    a = np.array([0,0])
    c = np.array([297,210])

    pm = tuple((c-a)/2)

    p1 = (-10,25,1)
    p2 = (10,25,1)
    p3 = (10,66,1)
    p4 = (-10,66,1)
    
    print("p1tr =", tuple(np.sum(rotation(30,pm) @ p1 * translation(pm[0],pm[1]), axis=1)))
    print("p2tr =", tuple(np.sum(rotation(30,pm) @ p2 * translation(pm[0],pm[1]), axis=1)))
    print("p3tr =", tuple(np.sum(rotation(30,pm) @ p3 * translation(pm[0],pm[1]), axis =1)))
    print("p4tr =", tuple(np.sum(rotation(30,pm) @ p4 * translation(pm[0],pm[1]), axis=1)))


def transform_coordinates(refs: np.ndarray, path: np.ndarray) -> np.ndarray:
    '''
    Transforms all coordinates in path base on refs.

    Parameters
    ----------
    refs: ndarray, shape (3, 2)
        A numpy array containing 3 corner points of the working space as reference.
        Point 1: lower right corner 
        Point 2: lower left corner 
        Point 3: top left corner
    path: ndarray, shape (number of path points, 2)
        A list of xy coordinates (shape: ).
        All points in the path are transformed based on refs.

    Returns
    -------
    path_trans: ndarray, shape (number of path points, 2)
        The transformed path, with all path points inside the rectangular area spanned by the reference points.
    '''
    # TODO: Add function body
    return np.zeros_like(path)
