try:
    from ulab import numpy as np
except ModuleNotFoundError:
    import numpy as np


def path_line(num_samples: int, xy_start: np.ndarray=np.array([0., 0.]), xy_end: np.ndarray=np.array([1., 1.])) -> np.ndarray:
    return np.linspace(xy_start, xy_end, num_samples)

def path_spiral(num_samples: int) -> np.ndarray:
    phi = (1 + 5**0.5)/2
    y0 = 1/(2+phi)
    x0 = (2*phi + 1)*y0
    theta0 = np.arctan2(-y0, -x0)
    k = 2*np.log(phi)/np.pi
    a = -x0 / (np.exp(k*theta0)*np.cos(theta0))

    t = np.linspace(-20, theta0, 1000)
    x = lambda t: x0 + a*np.exp(k*t)*np.cos(t)
    y = lambda t: y0 + a*np.exp(k*t)*np.sin(t)

    return np.array([x(t), y(t)]).T[::-1]
