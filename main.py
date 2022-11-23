#!/usr/bin/env pybricks-micropython

from two_link_arm import TwoLinkArm
from pybricks.parameters import Port
#from path_creation import path_line, path_spiral
# from coordinate_affine_transform import transform_coordinates


REFS = [
    [5, 5], # lower right
    [0, 0], # lower left
    [-5, 5], # upper left
]

params = dict(
    lengths=[ 13.7*10**-1, 9*10**-1],
    motor_speed=100,
    ports=[Port.A, Port.B],
    dist_threshold=0.1,
    smp_rate_measure=50, #ms
    smp_rate_target=10,
    jac_num_iter=50,
    jac_step_size=0.01,
    error_variant=1
)


def run_measuring():
    arm = TwoLinkArm(**params)
    arm.measure_coordinates()


def run_path_line():
    arm = TwoLinkArm(**params)
    # path = path_line(5) DEBUG
    path_trans = [[1,1],[-1,1],[-1,-1]] # , [0.00000000001,1], [-0.5,0.5]
    
    #path_trans = transform_coordinates(REFS, path)
    arm.motors[0].reset_angle(0)
    arm.motors[1].reset_angle(0)
    arm.follow_path(path_trans, "inverse")

def run_path_spiral():
    arm = TwoLinkArm(**params)
    path = path_spiral(200)    
    #path_trans = transform_coordinates(REFS, path)

    # arm.follow_path(path_trans, "jacobian")


# run_measuring()
run_path_line()
# run_path_spiral()
