from controller import Robot
import sys
sys.path.append('..')
from utils.motion_library import MotionLibrary


class Charlie (Robot):
    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        # there are 7 controllable LEDs on the NAO robot, but we will use only the ones in the eyes
        self.leds = {
            'rightf': self.getDevice('Face/Led/Right'), 
            'leftf': self.getDevice('Face/Led/Left'), 
            'righte': self.getDevice('Ears/Led/Right'), 
            'lefte': self.getDevice('Ears/Led/Left'), 
            'chest': self.getDevice('ChestBoard/Led'), 
        }

        self.library = MotionLibrary()
        # adding a custom motion to the library
        # self.library.add('Shove', './Shove.motion', loop=True)
        self.library.add('Anglehandupdown', './Anglehandupdown.motion', loop = True)

    def run(self):
        self.library.play('Stand')

        self.leds['rightf'].set(0x0000ff)
        self.leds['leftf'].set(0x0000ff)
        self.leds['righte'].set(0xff0000)
        self.leds['lefte'].set(0xff0000)
        self.leds['chest'].set(0xff0000)

        while self.step(self.time_step) != -1:
            # When the robot is done standing for stabilization, it moves forwards
            if self.library.get('Stand').isOver():
                self.library.play('ForwardLoop')  # walk forward
                self.library.play('Anglehandupdown')        # play the shove motion


# create the Robot instance and run main loop
wrestler = Charlie()
wrestler.run()
















