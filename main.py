#!/usr/bin/env pybricks-micropython
try:
    from ulab import numpy as np
except ModuleNotFoundError:
    import numpy as np

from two_link_arm import TwoLinkArm
from path_creation import path_line, path_spiral
from coordinate_affine_transform import transform_coordinates


path = path_line(200)
path = path_spiral(300)

refs = np.array([
    [5, 5],
    [0, 0],
    [-5, 5],
])

path_trans = transform_coordinates(refs, path)

arm = TwoLinkArm()

arm.measure_coordinates()

arm.follow_path(path_trans, "jacobian")
arm.follow_path(path_trans, "inverse")
#arm.to_coordinate(np.array([0, 0]), "inverse")
#arm.to_coordinate(np.array([0, 0]), "jacobian")

