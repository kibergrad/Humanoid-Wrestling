from controller import Robot
import sys
sys.path.append('..')
from utils.motion_library import MotionLibrary

# ##
class Dark_Lord (Robot):
    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

      # ##  there are 7 controllable LEDs on the NAO robot, but we will use only the ones in the eyes
        self.leds = {
            'right': self.getDevice('Face/Led/Right'),
            'left':  self.getDevice('Face/Led/Left')
        }

        self.library = MotionLibrary()
        self.library.add('Forward3Loop', './Forward3Loop.motion', loop=True)
        self.library.add('Cust', './Cust.motion', loop=True)

    def run(self):
        self.library.play('Stand')

        self.leds['right'].set(0xff0000) 
        self.leds['left'].set(0xff0000)

        while self.step(self.time_step) != -1:
            if self.library.get('Stand').isOver():
               self.library.play('Cust')        
               self.library.play('Forward3Loop') 

# ##create the Robot instance and run main loop
wrestler = Dark_Lord()
wrestler.run()

