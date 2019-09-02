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
import read
import num
#from picamera.array import PiRGBArray
from donkeycar.parts.camera import Webcam
from docopt import docopt

import donkeycar as dk
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

counter = stop = 0
stop_r = stop_b =True

class MyCVController:
    '''
    CV based controller
    '''
    def run(self, cam_img):
        
        lower_red = np.array([0, 114, 141])
        upper_red = np.array([19, 255, 255])
        lower_blue = np.array([103, 96, 132])
        upper_blue = np.array([118, 255, 255])
        lower_yellow = np.array([29,24,170])
        upper_yellow = np.array([36,255,255])
        outside = recording = False
        frame = cam_img
        global counter, stop_r, stop_b, stop
        steering = throttle = 0

        if cam_img is not None:
            
            img_hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
            '''
            mask_b = cv2.inRange(img_hsv, lower_blue, upper_blue)
            mask_r = cv2.inRange(img_hsv, lower_red, upper_red)
            # = mask0 + mask1
            '''
            frame = cv2.bitwise_and(frame, frame, mask=mask)
            '''
            frame_b = cv2.bitwise_and(frame, frame, mask=mask_b)
            frame_r = cv2.bitwise_and(frame, frame, mask=mask_r)
            '''
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #frame_b = cv2.cvtColor(frame_b, cv2.COLOR_BGR2GRAY)
            #frame_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
            frame = cv2.GaussianBlur(frame, (3, 3), 0)
            #frame_b = cv2.GaussianBlur(frame_b, (3, 3), 0)
            #frame_r = cv2.GaussianBlur(frame_r, (3, 3), 0)

            
            poly = [[0,20],[0,120],[160,120],[160,20]]
            mask = np.zeros_like(frame)
            #mask_b = np.zeros_like(frame_b)
            #mask_r = np.zeros_like(frame_r)
            cv2.fillPoly(mask,np.array([poly],'int32'),255)
            #cv2.fillPoly(mask_r,np.array([poly],'int32'),255)
            #cv2.fillPoly(mask_b,np.array([poly],'int32'),255)
            mask=cv2.bitwise_and(frame,mask)
            #mask_b=cv2.bitwise_and(frame_b,mask_b)
            #mask_r=cv2.bitwise_and(frame_r,mask_r)

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            '''
            contours_b, hierarchy_b = cv2.findContours(mask_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contours_r, hierarchy_r = cv2.findContours(mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            '''
            contours.sort(key=cv2.contourArea, reverse=True)
            '''
            contours_b.sort(key=cv2.contourArea, reverse=True)
            contours_r.sort(key=cv2.contourArea, reverse=True)
            '''
            #print("Red, Blue, stop" ,stop_r, stop_b, stop)
            no = num.MainTextRecognition(cam_img)
            print("NUM is = ", no)

            if(no == 1 and stop_b is True):
                #cb = contours_b[0]
                #if (cv2.contourArea(cb) > 150.0):
                #read.RemoveLocation('1')
                #print("blueArea =", cv2.contourArea(cb))
                stop_b=False
                counter = 0
                steering = throttle = 0
                stop = 1
                return steering, throttle, recording
            
            if (no == 2 and stop_r is True):
                #cr = contours_r[0]
                #if (cv2.contourArea(cr) > 150.0):
                #read.RemoveLocation('2')
                #print("redArea =", cv2.contourArea(cr))
                stop_r=False
                counter = 0
                steering = throttle = 0
                stop = 1
                return steering, throttle, recording
            
            if (stop):
                time.sleep(3)
                #for i in range(0,100000):
                    #print(i,throttle)
            
            if (len(contours) > 0):
                c = contours[0]
                print("trackArea = " ,cv2.contourArea(c))
                stop = 0
                counter += 1
                if (counter>200):
                    stop_r = True
                    stop_b = True
                    #stop_r = read.CheckLocation('2')
                    #stop_b = read.CheckLocation('1')
                    counter = 0
                
                if (cv2.contourArea(c) < 12.0):
                    outside = True
                else:
                    outside = False
                    counter += 1
                M = cv2.moments(c)
                try:
                    cx = int(M['m10'] / M['m00'])
                except:
                    cx = 80
                steering = (cx-80)/80.0 * 1.2
            else:
                outside = True
            throttle = 0.34
            if (outside):
                steering = 0.68
            #recording = True

        print("steer , throttle, count = " ,steering, throttle, counter)
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
    
    #global steering, throttle

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
