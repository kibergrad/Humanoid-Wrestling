
import sys
sys.path.append('..')
from utils.camera import Camera
from utils.fall_detection import FallDetection  # David's fall detection is implemented in this class
from utils.running_average import RunningAverage
from utils.image_processing import ImageProcessing as IP
from utils.finite_state_machine import FiniteStateMachine
from utils.current_motion_manager import CurrentMotionManager
from controller import Robot, Motion
import cv2
from utils.motion_library import MotionLibrary


class Dark_Lord (Robot):
    NUMBER_OF_DODGE_STEPS = 10

    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.time_step = int(self.getBasicTimeStep())

        self.fsm = FiniteStateMachine(
            states=['CHOOSE_ACTION', 'BLOCKING_MOTION'],
            initial_state='CHOOSE_ACTION',
            actions={
                'CHOOSE_ACTION': self.choose_action,
                'BLOCKING_MOTION': self.pending
            }
        )

        self.camera = Camera(self)
        self.library = MotionLibrary()
        self.library.add('Forward3Loop', './Forward3Loop.motion', loop=True)
        self.library.add('Cust3', './Cust3.motion', loop=True)





        # arm motors for getting up from a side fall
        self.RShoulderRoll = self.getDevice("RShoulderRoll")
        self.LShoulderRoll = self.getDevice("LShoulderRoll")

        self.fall_detector = FallDetection(self.time_step, self)
        self.current_motion = CurrentMotionManager()
        # load motion files
        self.motions = {
            'SideStepLeft': Motion('../motions/SideStepLeftLoop.motion'),
            'SideStepRight': Motion('../motions/SideStepRightLoop.motion'),
            'TurnRight': Motion('../motions/TurnRight20.motion'),
            'TurnLeft': Motion('../motions/TurnLeft20.motion'),
            'Kick': Motion('../controllers/participant/Cust3.motion'),
        }
        self.opponent_position = RunningAverage(dimensions=1)
        self.dodging_direction = 'left'
        self.counter = 0

    def run(self):
        while self.step(self.time_step) != -1:
            self.opponent_position.update_average(
                self._get_normalized_opponent_horizontal_position())
            self.fall_detector.check()
            self.fsm.execute_action()

    def choose_action(self):
        if self.opponent_position.average < -0.4:
            self.current_motion.set(self.motions['TurnLeft'])
            self.current_motion.set(self.motions['Kick'])
        elif self.opponent_position.average > 0.4:
            self.current_motion.set(self.motions['TurnRight'])
            self.current_motion.set(self.motions['Kick'])
        else:
            # dodging by alternating between left and right side steps to avoid easily falling off the ring
            if self.dodging_direction == 'left':
                if self.counter < self.NUMBER_OF_DODGE_STEPS:
                    self.current_motion.set(self.motions['SideStepLeft'])
                    self.current_motion.set(self.motions['Kick'])
                    self.counter += 1
                else:
                    self.dodging_direction = 'right'
                    self.current_motion.set(self.motions['Kick'])
            elif self.dodging_direction == 'right':
                if self.counter > 0:
                    self.current_motion.set(self.motions['SideStepRight'])
                    self.current_motion.set(self.motions['Kick'])
                    self.counter -= 1
                else:
                    self.dodging_direction = 'left'
                    self.current_motion.set(self.motions['Kick'])
            else:
                return
        self.fsm.transition_to('BLOCKING_MOTION')

    def pending(self):
        # waits for the current motion to finish before doing anything else
        if self.current_motion.is_over():
            self.fsm.transition_to('CHOOSE_ACTION')

    def _get_normalized_opponent_horizontal_position(self):
        """Returns the horizontal position of the opponent in the image, normalized to [-1, 1]
            and sends an annotated image to the robot window."""
        img = self.camera.get_image()
        largest_contour, vertical, horizontal = self.locate_opponent(img)
        output = img.copy()
        if largest_contour is not None:
            cv2.drawContours(output, [largest_contour], 0, (255, 255, 0), 1)
            output = cv2.circle(output, (horizontal, vertical), radius=2,
                                color=(0, 0, 255), thickness=-1)
        self.camera.send_to_robot_window(output)
        if horizontal is None:
            return 0
        return horizontal * 2 / img.shape[1] - 1

    def locate_opponent(self, img):
        """Image processing demonstration to locate the opponent robot in an image."""
        # we suppose the robot to be located at a concentration of multiple color changes (big Laplacian values)
        laplacian = cv2.Laplacian(img, cv2.CV_8U, ksize=3)
        # those spikes are then smoothed out using a Gaussian blur to get blurry blobs
        blur = cv2.GaussianBlur(laplacian, (0, 0), 2)
        # we apply a threshold to get a binary image of potential robot locations
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
        # the binary image is then dilated to merge small groups of blobs together
        closing = cv2.morphologyEx(
            thresh, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15)))
        # the robot is assumed to be the largest contour
        largest_contour = IP.get_largest_contour(closing)
        if largest_contour is not None:
            # we get its centroid for an approximate opponent location
            vertical_coordinate, horizontal_coordinate = IP.get_contour_centroid(
                largest_contour)
            return largest_contour, vertical_coordinate, horizontal_coordinate
        else:
            # if no contour is found, we return None
            return None, None, None


# create the Robot instance and run main loop
wrestler = Dark_Lord()
wrestler.run()


