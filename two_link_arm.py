import math
try:
    from ulab import numpy as np
except ModuleNotFoundError:
    import numpy as np
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch, DataLog


def get_error(x_old: np.ndarray, x_des: np.ndarray, x_cur: np.ndarray) -> float:
    # TODO: Implement function
    return 0.


def kin_forward(l, q):
    '''
    Uses the forward kinematics of a two-link arm to return the joint positions given two angles. 
    '''
    return np.array([[
        0, 0
    ],[
        l[0] * np.cos(q[0]), l[0] * np.sin(q[0]) 
    ],[
        l[0] * np.cos(q[0]) + l[1] * np.cos(np.sum(q)),
        l[0] * np.sin(q[0]) + l[1] * np.sin(np.sum(q)),
    ]]).T


def kin_jacobian(l, q):
    '''
    Computes the jacobian of the forward kinematic function of two-link arm. 
    '''
    return np.array([[
        -l[0]*np.sin(q[0]) - l[1]*np.sin(np.sum(q)),
        l[0]*np.cos(q[0]) + l[1]*np.cos(np.sum(q)),
    ],[
        -l[1]*np.sin(np.sum(q)),
        l[1]*np.cos(np.sum(q))
    ]]).T


class TwoLinkArm:
    l: np.ndarray
    motors: list
    motor_speed: float
    x_des_old: np.ndarray
    dist_threshold: float
    error: float
    num_error: int


    def __init__(self):
        self.l = np.array([0.75, 1]) 
        self.motor_speed = 100
        self.motors = [
            Motor(Port.A),
            Motor(Port.B)
        ]
        self.dist_threshold = 0.1  
        self.x_des_old = kin_forward(self.l, self._get_angles())[:, 2]
        self.error, self.num_error = 0., 0


    def follow_path(self, path: np.ndarray, method: str):
        watch = StopWatch()
        for x_des in path:
            self.to_coordinate(x_des, method)
        duration = watch.time()

        log = DataLog("error", "duration")
        log.log(self.error, duration)


    def to_coordinate(self, x_des: np.ndarray, method: str):
        if method == "inverse":
            q = self._to_coordinate_analytic(x_des)
        elif method == "jacobian":
            q = self._to_coordinate_jacobian(x_des, 50)
        else: 
            raise KeyError("Invalid method specification! Use 'inverse' or 'jacobian'")

    
    def _to_coordinate_analytic(self, x_des):
        l = self.l
        q = np.zeros(2)
        q[1] = np.arccos((np.sum(x_des**2)-np.sum(l**2)) / (2*np.product(l)))
        q[0] = np.arctan(x_des[1]/x_des[0]) - np.arctan((l[1]*np.sin(q[1])) / (l[0]+l[1]*np.cos(q[1])))

        # Correcting angle for 2nd and 3rd quadrant
        if x_des[0] < 0:
            q[0] -= np.pi

        self._set_angles(q)
        self._wait_till_target(x_des)
        self.x_des_old = x_des


    def _to_coordinate_jacobian(self, x_des, num_iter):
        step_size = 0.01
        l = self.l
        for i in range(num_iter):
            q = self._get_angles()
            x_cur = kin_forward(l, q)[:, 2]
            inv_jacobian = np.linalg.pinv(kin_jacobian(l, q)) # TODO: pinv maybe not available in ulab
            delta_q = inv_jacobian @ (x_des - x_cur)[:, None]
            q += step_size * delta_q[:, 0]

            # Forcing second joint angle to be positive for stability
            if q[1] <= 0: 
                q[1] = 1e-3

            self._set_angles(q)
            self._wait_till_target(kin_forward(self.l, q)[:, 2])


    def _set_angles(self, rad):
        for i in range(2):
            self.motors[i].run_target(self.motor_speed, math.degrees(rad[i]))


    def _get_angles(self):
        return np.array([math.radians(motor.angle()) for motor in self.motors])


    def _wait_till_target(self, x_des):
        time_sample = 10 # ms
        x_cur = kin_forward(self.l, self._get_angles())[:, 2]
        while np.linalg.norm(x_cur - x_des) > self.dist_threshold:
            # Compute normal distance to connection line as error
            error = get_error(self.x_des_old, x_des, x_cur) # TODO: Create function
            self.num_error += 1
            self.error = self.error * (self.num_error-1) / self.num_error + error / self.num_error

            wait(time_sample) 
            x_cur = kin_forward(self.l, self._get_angles())[:, 2]
            

