
from controller import Robot
import sys
sys.path.append('..')
from utils.motion_library import MotionLibrary
from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.gait_manager import GaitManager
from utils.camera import Camera
from utils.finite_state_machine import FiniteStateMachine
from boundaryDetection import boundaryDetection as BD

class Sultaan (Robot):
    SMALLEST_TURNING_RADIUS = 0.1 #0.1
    SAFE_ZONE = 0.75
    TIME_BEFORE_DIRECTION_CHANGE = 40  # 80

    def __init__(self):
        Robot.__init__(self)
        self.fall = False
        self.time_step = int(self.getBasicTimeStep())
        self.library = MotionLibrary()

        self.camera = Camera(self)
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        self.counter = 0
        self.library.add('Anglehandupdown', './Shove.motion', loop = True)
        # self.library.add('Anglehandupdown', './.motion', loop = True)
        self.leds = {
            'rightf': self.getDevice('Face/Led/Right'), 
            'leftf': self.getDevice('Face/Led/Left'), 
            'righte': self.getDevice('Ears/Led/Right'), 
            'lefte': self.getDevice('Ears/Led/Left'), 
            'chest': self.getDevice('ChestBoard/Led'), 
        }


    def run(self):
        while self.step(self.time_step) != -1:
            t = self.getTime()
            self.leds['rightf'].set(0xff0000)
            self.leds['leftf'].set(0xff0000)
            self.leds['righte'].set(0xff0000)
            self.leds['lefte'].set(0xff0000)
            self.leds['chest'].set(0xff0000)
            self.gait_manager.update_theta()
            if 0.3 < t < 1:
                self.start_sequence()
            elif t > 1:
                self.fall_detector.check()
                self.walk()

    def start_sequence(self):
        self.gait_manager.command_to_motors(heading_angle=0)
        
    def boundaryDetection(self):
        img = self.camera.get_image()
        distance = BD.getDistance(img)
        return distance

    def walk(self):
        normalized_x = self._get_normalized_opponent_x() 
        desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x) if abs(normalized_x) > 1e-3 else None
        if(normalized_x != -1): 
            # dist = self.boundaryDetection() 
            # print(dist)
            # if(dist <20):
            #     self.heading_angle = 3.14/6
            #     self.gait_manager.command_to_motors(desired_radius=desired_radius, heading_angle=self.heading_angle)
                
            # else: 
            self.heading_angle = 0
            self.gait_manager.command_to_motors(desired_radius=0, heading_angle=self.heading_angle)

        self.counter += 1

    def _get_normalized_opponent_x(self):
        img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            return 0
        return horizontal_coordinate * 2 / img.shape[1] - 1

wrestler = Sultaan()
wrestler.run()
