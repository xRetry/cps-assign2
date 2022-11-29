import math

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch, DataLog


def kin_forward(l, q, scale):
    '''
    Uses the forward kinematics of a two-link arm to return the joint positions given two angles. 
    '''
    return [ 
        l[0] * math.cos(q[0]) + l[1] * math.cos(sum(q)) * scale, 
        l[0] * math.sin(q[0]) + l[1] * math.sin(sum(q)) * scale
    ]
          

def vecnorm(v1:list, v2:list):
    return math.sqrt((v1[0] - v2[0])**2 + ((v1[1] - v2[1])**2))


class TwoLinkArm:
    
    def __init__(self, lengths=[0.75, 1], motor_speed=100, ports=[Port.A, Port.B], 
    dist_threshold=0.1, smp_rate_measure=50, smp_rate_target=10, calibrate=False):
        self.ev3 = EV3Brick()
        self.l = lengths
        self.motor_speed = motor_speed
        self.motors = [
            Motor(ports[0]),
            Motor(ports[1])
        ]
        self.dist_threshold = dist_threshold
        self.x_des_old = kin_forward(self.l, self._get_angles(), 1)
        self.error, self.num_error = 0., 0
        self.smp_rate_measure = smp_rate_measure
        self.smp_rate_target = smp_rate_target

        if calibrate:
            self.coord_scale = sum(self.l) / self.x_des_old[0]


    def measure_coordinates(self):
        while True:
            # Should print the coodinates and angles if any button is pressed
            if len(self.ev3.buttons.pressed()) > 0:
                q = self._get_angles()
                xy = kin_forward(self.l, q, self.coord_scale)

                self.ev3.screen.clear()
                self.ev3.screen.print("q1: ", math.degrees(q[0]))
                self.ev3.screen.print("q2: ", math.degrees(q[1]))
                self.ev3.screen.print("x: ", xy[0])
                self.ev3.screen.print("y: ", xy[1])
            wait(self.smp_rate_measure)


    def follow_path(self, path: list):
        watch = StopWatch()
        for x_des in path:
            print("position init", x_des)
            self.to_coordinate(x_des)
            print("position done", x_des)
        duration = watch.time()

        log = DataLog("error", "duration")
        log.log(self.error, duration)

    
    def to_coordinate(self, x_des):
        l = self.l
        q = [0., 0.]
        sum_temp = (sum([x**2 for x in x_des])- sum([x**2 for x in l])) / (2*l[0]*l[1])
        if(sum_temp >=1 ):
            sum_temp = 1
        if(sum_temp <=-1):
            sum_temp = -1
        q[1] = math.acos(sum_temp)
        
        q[0] = math.atan(x_des[1]/x_des[0]) - math.atan((l[1]*math.sin(q[1])) / (l[0]+l[1]*math.cos(q[1])))

        # Correcting angle for 2nd and 3rd quadrant
        if x_des[0] < 0:
            q[0] -= math.pi

        self._set_angles(q)
        wait(1000)
        # self._wait_till_target(x_des)
        self.x_des_old = x_des


    def _set_angles(self, rad):
        rad[0] = (rad[0] % (2*math.pi))
        rad[1] = -(rad[1] % (2*math.pi))

        for i in range(2):
            self.motors[i].run_target(
                speed=self.motor_speed, 
                target_angle=math.degrees(rad[i]),
                wait=False # Otherwise this would pause the program
            )


    def _get_angles(self):
        q= [math.radians(motor.angle()) for motor in self.motors]
        q[1] = -q[1]

        return q


    def _wait_till_target(self, x_des):
        x_cur = kin_forward(self.l, self._get_angles(), self.coord_scale)
        
        while vecnorm(x_cur, x_des) > self.dist_threshold:
            print("in vecnorm loop", x_cur, x_des)

            '''
            [ ERROR DERIVATION ] [ VARIANT 1]
            '''

            error = self._get_error(self.x_des_old, x_des, x_cur)
            self.num_error += 1
            self.error = self.error * (self.num_error-1) / self.num_error + error / self.num_error            

            wait(self.smp_rate_target) 
            x_cur = kin_forward(self.l, self._get_angles(), self.coord_scale)

    
    def _get_error(self, x_old: list, x_des: list, x_cur: list) -> float:
        error = 0.0
        """
        '''
        #[ VARIANT 1 ] Calculates the measured absolute distance between mid-point of x_old <-> x_des AND x_cur in coordinate Format
        #Thought about using orthogonal vectors, but we still assume x/y ranges to be in reasonable space (x_cur) between x_old and x_dest
        #Let's see how relative distance from point performs. If bad, i would straight advise to go for regression analysis in time-series manner.
        '''
        x_mid = (x_old[0] + x_des[0])/2
        y_mid = (x_old[1] + x_des[1])/2
        midpoint = [[x_mid,y_mid]]

        error = math.dist(x_cur,midpoint)
        """
        return error
            

