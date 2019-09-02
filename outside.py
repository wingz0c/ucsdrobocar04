#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    manage.py (drive)


Options:
    -h --help          Show this screen.
"""
import os
import time
import numpy as np
import cv2
import threading
from circle import detectcircle

#from picamera.array import PiRGBArray
from donkeycar.parts.camera import Webcam
from docopt import docopt

import donkeycar as dk
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

#lower_red1 = np.array([0, 70, 70])
#upper_red1 = np.array([10, 255, 255])


# upper mask (170-180)
#lower_red2 = np.array([170, 70, 70])
#upper_red2 = np.array([180, 255, 255])
lower_yellow = np.array([29,24,170])
upper_yellow = np.array([36,255,255])
class MyCVController:
    '''
    CV based controller
    '''
    def run(self, cam_img):

        steering = 0
        throttle = 0
        recording = False
        frame = cam_img
        outside = False
        

        if cam_img is not None:
            #[x,y] ,RGB = detectcircle(cam_img)
            #print("[x,y], RGB = ",[x,y], RGB)
            
            img_hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
            #mask0 = cv2.inRange(img_hsv, lower_red1, upper_red1)
            #mask1 = cv2.inRange(img_hsv, lower_red2, upper_red2)
            # = mask0 + mask1

            frame = cv2.bitwise_and(frame, frame, mask=mask)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (3, 3), 0)
            
            poly = [[0,40],[0,110],[160,110],[160,40]]
            mask = np.zeros_like(blur)
            cv2.fillPoly(mask,np.array([poly],'int32'),255)
            mask=cv2.bitwise_and(blur,mask)
            
            #ret, thresh = cv2.threshold(blur, 70, 255, cv2.THRESH_BINARY_INV)
            #contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_SIMPLE)
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

            contours.sort(key=cv2.contourArea, reverse=True)
            if (len(contours) > 0):
                c = contours[0]
                print(cv2.contourArea(c))
                if (cv2.contourArea(c) < 9.0):
                    outside = True
                M = cv2.moments(c)
                try:
                    cx = int(M['m10'] / M['m00'])
                except:
                    cx = 80
                steering = (cx-80)/80.0 * 1.2
            else:
                outside = True
            if (outside):
                steering = 0.7
            throttle = 0.4
            recording = True
        print("steer , throttle = " ,steering, throttle)
        return steering, throttle, recording


def drive(cfg):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    '''

    # Initialize car
    V = dk.vehicle.Vehicle()

    # Camera
    cam = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
    V.add(cam, outputs=['cam/image_array'], threaded=True)

    # Controller
    V.add(MyCVController(),
          inputs=['cam/image_array'],
          outputs=['steering', 'throttle', 'recording'])

    # Sombrero
    if cfg.HAVE_SOMBRERO:
        from donkeycar.parts.sombrero import Sombrero
        s = Sombrero()

    # Drive train setup

    from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

    steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    steering = PWMSteering(controller=steering_controller,
                           left_pulse=cfg.STEERING_LEFT_PWM,
                           right_pulse=cfg.STEERING_RIGHT_PWM)

    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    throttle = PWMThrottle(controller=throttle_controller,
                           max_pulse=cfg.THROTTLE_FORWARD_PWM,
                           zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                           min_pulse=cfg.THROTTLE_REVERSE_PWM)

    V.add(steering, inputs=['steering'])
    V.add(throttle, inputs=['throttle'])

    # add tub to save data

    inputs = ['cam/image_array',
              'steering', 'throttle']

    types = ['image_array',
             'float', 'float']

    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=inputs, types=types)
    V.add(tub, inputs=inputs, outputs=["tub/num_records"], run_condition='recording')

    # run the vehicle
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ,
            max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()

    if args['drive']:
        drive(cfg)
    """
    while True:
        # k = cv2.waitKey(0) & 0xFF
        k = input('input something!: ')
        if k == ord('w'):
            self.throttle += 0.05
        elif k == ord('s'):
            self.throttle -= 0.05
        elif k == ord('p'):
            self.throttle = 0
    """
