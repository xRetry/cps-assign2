#!/usr/bin/env pybricks-micropython
try:
    from ulab import numpy as np
except ModuleNotFoundError:
    import numpy as np

from two_link_arm import TwoLinkArm
from path_creation import path_line, path_spiral
from coordinate_affine_transform import transform_coordinates


REFS = np.array([
    [5, 5], # lower right
    [0, 0], # lower left
    [-5, 5], # upper left
])


def run_measuring():
    arm = TwoLinkArm()
    arm.measure_coordinates()


def run_path_line():
    arm = TwoLinkArm()
    path = path_line(5)
    path_trans = transform_coordinates(REFS, path)

    arm.follow_path(path_trans, "inverse")

def run_path_spiral():
    arm = TwoLinkArm()
    path = path_spiral(200)
    path_trans = transform_coordinates(REFS, path)

    arm.follow_path(path_trans, "inverse")


run_measuring()
#run_path_line()
#run_path_spiral()
