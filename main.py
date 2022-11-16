#!/usr/bin/env pybricks-micropython
try:
    from ulab import numpy as np
except ModuleNotFoundError:
    import numpy as np

from two_link_arm import TwoLinkArm


arm = TwoLinkArm()
arm.to_coordinate(np.array([0, 0]), "inverse")
arm.to_coordinate(np.array([0, 0]), "jacobian")

