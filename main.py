#!/usr/bin/env pybricks-micropython
try:
    from ulab import numpy as np
except ModuleNotFoundError:
    import numpy as np

from two_link_arm import TwoLinkArm
from path_creation import path_line, path_spiral

path = path_line(200)
path = path_spiral(300)

arm = TwoLinkArm()
arm.follow_path(path, "jacobian")
arm.follow_path(path, "inverse")
#arm.to_coordinate(np.array([0, 0]), "inverse")
#arm.to_coordinate(np.array([0, 0]), "jacobian")

