import sys
from controller import Robot
sys.path.append('..') # adding the utils folder to get access to some custom helper functions, have a look at it
from utils.motion_library import MotionLibrary


class Bob (Robot):
    def __init__(self):
        super().__init__()
        # to load all the motions from the motion folder, we use the Motion_library class:
        self.library = MotionLibrary()

        # we initialize the shoulder pitch motors using the Robot.getDevice() function:n b
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

    def run(self):
        # to control a motor, we use the setPosition() function:
        self.RShoulderPitch.setPosition(1.3)
        self.LShoulderPitch.setPosition(1.3)
        # for more motor control functions, see the documentation: https://cyberbotics.com/doc/reference/motor
        # to see the list of available devices, see the NAO documentation: https://cyberbotics.com/doc/guide/nao

        time_step = int(self.getBasicTimeStep())
        while self.step(time_step) != -1:
            if self.getTime() == 1: # We wait a bit for the robot to stabilise
                # to play a motion from the library, we use the play() function as follows:
                self.library.play('Backwards')


# create the Robot instance and run main loop
wrestler = Bob()
wrestler.run()
