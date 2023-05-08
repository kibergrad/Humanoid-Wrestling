from controller import Robot
import sys
sys.path.append('..')
from utils.motion_library import MotionLibrary


class Dark_Lord (Robot):
    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        # there are 7 controllable LEDs on the NAO robot, but we will use only the ones in the eyes
        self.leds = {
            'right': self.getDevice('Face/Led/Right'),
            'left':  self.getDevice('Face/Led/Left')
        }

        self.library = MotionLibrary()
        # adding a custom motion to the library
        # self.library.add('Shove', './Shove.motion', loop=True)

    def run(self):
        self.library.play('Stand')

        self.leds['right'].set(0xff0000)  # set the eyes to red
        self.leds['left'].set(0xff0000)

        while self.step(self.time_step) != -1:
            # When the robot is done standing for stabilization, it moves forwards
            if self.library.get('Stand').isOver():
                self.library.play('ForwardLoop')  # walk forward
                self.library.play('Custom')        # play the shove motion


# create the Robot instance and run main loop/
wrestler = Dark_Lord()
wrestler.run()
