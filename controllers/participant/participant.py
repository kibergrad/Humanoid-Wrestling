from controller import Robot
import sys
sys.path.append('..')
from myutils.motion_library import MotionLibrary

# Eve's locate_opponent() is implemented in this module:
from myutils.image_processing import ImageProcessing as IP
from myutils.fall_detection import FallDetection
from myutils.gait_manager import GaitManager
from myutils.camera import Camera
from myutils.camera2 import Camera2
from myutils.finite_state_machine import FiniteStateMachine
from myutils.ellipsoid_gait_generator import EllipsoidGaitGenerator


import torch
import cv2
from torchvision import transforms
import numpy as np
import time
import threading
    # Load the YOLOv5 model


    # Create an OpenCV window for displaying the YOLOv5 detection results
# cv2.namedWindow('YOLOv5 Detection', cv2.WINDOW_NORMAL)
# cv2.resizeWindow('YOLOv5 Detection', 800, 600)

model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/ankit/IROS/controllers/participant/recent.pt')
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model.to(device).eval()





class Sultaan (Robot):
    SMALLEST_TURNING_RADIUS = 0.1 #0.1
    SAFE_ZONE = 0.75
    TIME_BEFORE_DIRECTION_CHANGE = 60   # 80
    k=0
    is_bot_visible = True
    
    
    def __init__(self):
        Robot.__init__(self)
        self.fall = Falsed = 0
        
        self.time_step = int(self.getBasicTimeStep())
        self.library = MotionLibrary()

        self.camera = Camera(self)
        self.camera2 = Camera2(self)
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        self.counter = 0
        #self.library.add('Anglehandupdown', './First.motion', loop = True)
        self.leds = {
            'rightf': self.getDevice('Face/Led/Right'), 
            'leftf': self.getDevice('Face/Led/Left'), 
            'righte': self.getDevice('Ears/Led/Right'), 
            'lefte': self.getDevice('Ears/Led/Left'), 
            'chest': self.getDevice('ChestBoard/Led'), 
        }
        
        self.HeadPitch = self.getDevice("HeadPitch")
       
        #self.library.play('Cust')
        # for locking motor
       
    def run(self):
        k=0
        
        
        yolo_thread = threading.Thread(target=self.run_yolo)
        yolo_thread.start()
        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            #self.HeadPitch.setPosition(0)
            t = self.getTime()
            self.leds['rightf'].set(0xff0000)
            self.leds['leftf'].set(0xff0000)
            self.leds['righte'].set(0xff0000)
            self.leds['lefte'].set(0xff0000)
            self.leds['chest'].set(0xff0000)
            self.gait_manager.update_theta()
            #x, k, z, yaw = EllipsoidGaitGenerator.compute_leg_position(self, is_left = 'True', desired_radius=1e3, heading_angle=0)
            #print('x=' + str(x))
            
            if(self.fall_detector.detect_fall()): 
                self.fall = True
            if 0.3 < t < 5:
                self.start_sequence()
            elif t > 5:
                
                self.fall
                self.fall_detector.check()
                
                if(not self.fall):
                    #print('t_before_yolo: {:.6f}'.format(round(t, 6)))
                    
                    self.walk()
                    d = self.getDistance()
                    if d == 1:
                    # print("boundary overflow")
                    #prevD = d
                    # self.heading_angle = 3.14 / 2
                        self.library.play('TurnLeft60')
                    else:
                        #self.yolo()
                        self.walk()

    
    def getDistance(self):          #we use bottom oriented image for edge detection
        import cv2
        import numpy as np
        image = self.camera2.get_image()
        m = 0
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = (0, 50, 50)
        upper_red = (10, 255, 255)
        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
       
        # print('len(image.getbands()):',len(image.getbands()))
        # image_shape = image.shape

# Get the number of channels from the shape tuple
        # num_channels = image_shape[-1]

        # print('num_channels:', num_channels)
        
        rgb_image = image[:, :, :3]

# Get the shape of the RGB image
        rgb_image_shape = rgb_image.shape

# Get the number of channels from the shape tuple
        num_channels = rgb_image_shape[-1]
        print('num_channels:', num_channels)
        
        
        
        
        
        
        
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(largest_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            image_height, image_width = image.shape[:2]
            bottom_threshold = 0.92 * image_height
            
            for point in box:
                x, y = point
                if y >= bottom_threshold:
                    print("Point:", point)
                    print("Bottom Threshold:", bottom_threshold)

            points_below_threshold = sum(point[1] >= bottom_threshold for point in box)
            percentage_below_threshold = points_below_threshold / len(box)
            
            #if any(point[1] >= bottom_threshold for point in box):
            cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
            print('percentage_below_threshold: ', percentage_below_threshold)
            if percentage_below_threshold >= 0.5:    #print('point[1]: ', point)
                #print('bottom_threshold: ', bottom_threshold)
                if cv2.contourArea(largest_contour) >= 200:
                    print("Turn to avoid falling!")
                    
                    m=1
                
                else:
                    print("No need to turn, keep moving.")
            else:
                print("No need to turn, keep moving.")
        else:
            print("No red contours found, keep moving.")
        return m

    
    
    def run_yolo(self):
        time.sleep(2)
        while True:
            # Capture the image from the camera
            image = self.camera.get_image()

            # Remove alpha channel if present
            if image.shape[2] == 4:
                image = image[:, :, :3]

            # Convert image to RGB format
            img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Perform object detection
            results = model([img])

            # Display the detections
            results.print()

            # Access individual detection attributes (e.g., bounding boxes, labels)
            boxes = results.xyxy[0].numpy()
            labels = results.names[0]

            # Process the detection results as needed
            if len(boxes) == 0:
                is_bot_visible = False
                self.library.play('TurnLeft60')
            else:
                is_bot_visible = True
                

            # You can perform further actions based on the detection results

            # Sleep for a short duration to avoid excessive CPU usage
            time.sleep(0.1)
    
    
    
  
    def start_sequence(self):
        """At the beginning of the match, the robot walks forwards to move away from the edges."""
        self.gait_manager.command_to_motors(heading_angle=0)
        
    
    
    def walk(self):
        normalized_x = self._get_normalized_opponent_x() 
        desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x) if abs(normalized_x) > 1e-3 else None
        if(normalized_x > 0): 
            self.heading_angle = 3.14/4
            self.counter = 0;  
        elif(normalized_x < 0): 
            self.heading_angle = -(3.14/4)
            self.counter = 0 
        elif(normalized_x == 0): 
            return  
        self.counter += 1
        self.gait_manager.command_to_motors(desired_radius=desired_radius, heading_angle=self.heading_angle)
        #self.library.play('Khushi')

    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            return 0
        return horizontal_coordinate * 2 / img.shape[1] - 1

# create the Robot instance and run main loop
wrestler = Sultaan()
wrestler.run()

