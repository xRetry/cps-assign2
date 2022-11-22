class Motor:

    def __init__(self, *args, **kwargs):
        self._angle = 0

    def run_target(self, speed, target_angle, *args, **kwargs):
        self._angle = target_angle 

    def angle(self):
        return self._angle
    
