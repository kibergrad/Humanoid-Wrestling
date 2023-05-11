from controller import Robot
import sys
sys.path.append('..')
from utils.motion_library import MotionLibrary

# Eve's locate_opponent() is implemented in this module:
from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.gait_manager import GaitManager
from utils.camera import Camera


class Dark_Lord (Robot):
    SMALLEST_TURNING_RADIUS = 0.01
    SAFE_ZONE = 0.75
    TIME_BEFORE_DIRECTION_CHANGE = 0  # 8000 ms / 40 ms/

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())
        self.library = MotionLibrary()

        self.camera = Camera(self)
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        self.counter = 0
        self.library.add('Anglehandupdown', './Anglehandupdown.motion', loop = True)
        self.leds = {
            'rightf': self.getDevice('Face/Led/Right'), 
            'leftf': self.getDevice('Face/Led/Left'), 
            'righte': self.getDevice('Ears/Led/Right'), 
            'lefte': self.getDevice('Ears/Led/Left'), 
            'chest': self.getDevice('ChestBoard/Led'), 
        }
    def run(self):
        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            t = self.getTime()
            # self.LeftHand.setPosition(-0.3)
            # self.RightHand.setPosition(0.3)
            self.leds['rightf'].set(0xff0000)
            self.leds['leftf'].set(0xff0000)
            self.leds['righte'].set(0xff0000)
            self.leds['lefte'].set(0xff0000)
            self.leds['chest'].set(0xff0000)

            self.gait_manager.update_theta()
            if 0.3 < t < 3:
                self.start_sequence()
            elif t > 3:
                self.fall_detector.check()
                self.walk()

    def start_sequence(self):
        """At the beginning of the match, the robot walks forwards to move away from the edges."""
        self.gait_manager.command_to_motors(heading_angle=0)

    def walk(self):
        normalized_x = self._get_normalized_opponent_x()
        desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x) if abs(normalized_x) > 1e-3 else None
        if self.heading_angle > 0 :
            if self.counter > self.TIME_BEFORE_DIRECTION_CHANGE:
                self.heading_angle = - self.heading_angle
                self.counter = 0 
        if self.heading_angle < 0 : 
            if self.counter > self.TIME_BEFORE_DIRECTION_CHANGE: 
                self.heading_angle = 0
                self.counter = 0
        self.counter += 1
        self.gait_manager.command_to_motors(desired_radius=desired_radius, heading_angle=self.heading_angle)
        self.library.play('Anglehandupdown')

    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            return 0
        return horizontal_coordinate * 2 / img.shape[1] - 1

# create the Robot instance and run main loop
wrestler = Dark_Lord()
wrestler.run()

