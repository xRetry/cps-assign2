import math
try:
    from ulab import numpy as np
except ModuleNotFoundError:
    import numpy as np
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch, DataLog


def get_error(x_old: np.ndarray, x_des: np.ndarray, x_cur: np.ndarray, variant: int) -> float:
    error = 0.0
    if(variant==1):
        '''
        [ VARIANT 1 ] Calculates the measured absolute distance between mid-point of x_old <-> x_des AND x_cur in coordinate Format
        Thought about using orthogonal vectors, but we still assume x/y ranges to be in reasonable space (x_cur) between x_old and x_dest
        Let's see how relative distance from point performs. If bad, i would straight advise to go for regression analysis in time-series manner.
        '''
        x_mid = (x_old[0] + x_des[0])/2
        y_mid = (x_old[1] + x_des[1])/2
        midpoint = np.array([[x_mid,y_mid]])

        error = math.dist(x_cur,midpoint)
        
    if(variant==2):
        '''
        [ VARIANT 2 ] Classic line regression analysis (area under-/over between x_curr, x_des)
        '''
        y_cur = []
        for coordinate in x_cur:
            y_cur.append(coordinate[1])
        
        y_dest = []
        for coordinate in x_dest:
            y_dest.append(coordinate[1])
        
        
        '''
        Raw Error | Absolute Error | Relative Error | Percentage Error | Mean Absolute Error | Mean Squared Error (MSE) | Root Mean Squared Error (RMSE)
        '''
        error_raw = y_cur - y_dest
        error_abs = np.abs(y_cur - y_dest)
        error_rel = np.divide(np.abs(y_cur - y_dest), y_cur)
        error_perc = error_rel * 100
        error_MAE = np.mean(np.abs(y_cur-y_dest))
        error_MSE = np.mean(np.square(y_cur-y_dest))
        error_RMSE = np.root(error_MSE)
        
        ## --> print values then or write to file. @ Marco -- did you find a filewriter ? or should we use the logger from ev3 for that purpose ?


    return error


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
    ev3: EV3Brick
    motors: list
    motor_speed: float
    x_des_old: np.ndarray
    dist_threshold: float
    error: float
    num_error: int
    x_cur_ALL: list


    def __init__(self):
        self.ev3 = EV3Brick()
        self.l = np.array([0.75, 1]) 
        self.motor_speed = 100
        self.motors = [
            Motor(Port.A),
            Motor(Port.B)
        ]
        self.dist_threshold = 0.1  
        self.x_des_old = kin_forward(self.l, self._get_angles())[:, 2]
        self.error, self.num_error = 0., 0


    def measure_coordinates(self):
        while True:
            # Should print the coodinates and angles if any button is pressed
            if len(self.ev3.buttons.pressed()) > 0:
                q = self._get_angles()
                xy = kin_forward(self.l, q)[:, 2]

                self.ev3.screen.clear()
                self.ev3.screen.print("q1: ", math.degrees(q[0]))
                self.ev3.screen.print("q2: ", math.degrees(q[1]))
                self.ev3.screen.print("x: ", xy[0])
                self.ev3.screen.print("y: ", xy[1])
            wait(50)


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
            '''
            [ ERROR DERIVATION ] [ VARIANT 1]
            '''

            error = get_error(self.x_des_old, x_des, x_cur, 1)
            self.num_error += 1
            self.error = self.error * (self.num_error-1) / self.num_error + error / self.num_error            

            '''
            [ ERROR DERIVATION ] [ VARIANT 2]
            '''
            self.x_cur_ALL.append(x_cur)
            ## [ CONTINUATION : ] run once at end of script


            wait(time_sample) 
            x_cur = kin_forward(self.l, self._get_angles())[:, 2]
            

