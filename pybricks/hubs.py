class Screen:
    def clear(self):
        pass

    def print(self, *args, **kwargs):
        pass

class Buttons:
    def pressed(self):
        return [1]
    

class EV3Brick:
    buttons: any
    screeen: any

    def __init__(self):
        self.screen = Screen()
        self.buttons = Buttons()
    
