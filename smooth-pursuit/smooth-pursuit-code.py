import picamera
import cv2
import imutils
import numpy as np
import threading
import time

from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS

import sys
import RPi.GPIO as gp
import os
gp.setwarnings(False)
gp.setmode(gp.BOARD)

# initialize GPIOs
global redLed
redLed = 21
global panPinL
panPinL = 37
global tiltPinL
tiltPinL = 35
global panPinR
panPinR = 40
global tiltPinR
tiltPinR = 36
global colorLower
colorLower = (110, 50, 50)
global colorUpper
colorUpper = (130, 255, 255)

global pr
pr = 90
global tr
tr = 90
global pl
pl = 90
global tl 
tl = 90
global RightpanServoAngle
RightpanServoAngle = 90
global RighttiltServoAngle
RighttiltServoAngle = 90

global LeftpanServoAngle
LeftpanServoAngle = 90
global LefttiltServoAngle
LefttiltServoAngle = 90
# See IvPort documentation for information on GPIO pins
gp.setup(7, gp.OUT)  # Selection pin
gp.setup(11, gp.OUT) # Enable1 pin
gp.setup(12, gp.OUT) # Enable2 pin

def set_camera(i):

    if i==1:
        gp.output(7, False)
        gp.output(11, False)
        gp.output(12, True)

    elif i==2:
        gp.output(7, False)
        gp.output(11, True)
        gp.output(12, False)

    else:
        print ("set_camera: Invalid camera number given, active camera port was not changed.")


def setServoAngle(PanR,PanPinR,PanL,PanPinL,TiltR,TiltPinR,TiltL,TiltPinL):
    #assert PanR >=30 and PanR <= 150
    #assert PanL >=30 and PanL <= 150
    #assert TiltR >=30 and TiltR <= 150
    #assert TiltL >=30 and TiltL <= 150
    pwm1 = gp.PWM(PanPinR,50)
    pwm2 = gp.PWM(PanPinL,50)
    pwm3 = gp.PWM(TiltPinR,50)
    pwm4 = gp.PWM(TiltPinL,50)

    pwm1.start(6)
    pwm2.start(6)
    pwm3.start(6)
    pwm4.start(6)
    
    dutyCycle1 = PanR / 18. + 3.
    dutyCycle2 = PanL / 18. + 3.
    dutyCycle3 = TiltR / 18. + 3.
    dutyCycle4 = TiltL / 18. + 3.
    
    pwm1.ChangeDutyCycle(dutyCycle1)
    pwm2.ChangeDutyCycle(dutyCycle2)
    pwm3.ChangeDutyCycle(dutyCycle3)
    pwm4.ChangeDutyCycle(dutyCycle4)
    
    pwm1.stop()
    pwm2.stop()
    pwm3.stop()
    pwm4.stop()
    gp.cleanup()
        

def LeftservoPosition (xl, yl):
    global LeftpanServoAngle
    global LefttiltServoAngle
    if (xl < 229):
        LeftpanServoAngle -= 10
        if LeftpanServoAngle > 150:
            LeftpanServoAngle = 150
  
    if (xl > 269):
        LeftpanServoAngle += 10
        if LeftpanServoAngle < 30:
            LeftpanServoAngle = 30

    if (yl < 167):
        LefttiltServoAngle -= 10
        if LefttiltServoAngle > 150:
            LefttiltServoAngle = 150
  
    if (yl > 207):
        LefttiltServoAngle += 10
        if LefttiltServoAngle < 30:
            LefttiltServoAngle = 30
    
def RightservoPosition (xr, yr):
    global RightpanServoAngle
    global RighttiltServoAngle
    if (xr < 229):
        RightpanServoAngle -= 10
        if RightpanServoAngle > 150:
            RightpanServoAngle = 150
  
    if (xr > 269):
        RightpanServoAngle += 10
        if RightpanServoAngle < 30:
            RightpanServoAngle = 30

    if (yr < 167):
        RighttiltServoAngle += 10
        if RighttiltServoAngle > 150:
            RighttiltServoAngle = 150
  
    if (yr > 207):
        RighttiltServoAngle -= 10
        if RighttiltServoAngle < 30:
            RighttiltServoAngle = 30

def RightCamera(image):
    global pr
    global tr
    image1 = imutils.resize(image, width=500)
    hsv1 = cv2.cvtColor(image1, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv1, colorLower, colorUpper)
    mask1 = cv2.erode(mask1, None, iterations=2)
    mask1 = cv2.dilate(mask1, None, iterations=2)
    cnts1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts1 = cnts1[0] if imutils.is_cv2() else cnts1[1]
    center = None
    if len(cnts1) > 0:
        c = max(cnts1, key=cv2.contourArea)
        ((xr, yr), radiusr) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radiusr > 10:
            cv2.circle(image1, (int(xr), int(yr)), int(radiusr),
                       (0, 255, 255), 2)
            pr,tr = RightservoPosition(int(xr), int(yr))
            cv2.circle(image1, center, 5, (0, 0, 255), -1)
    return image1
    
def LeftCamera(image):
    global pl
    global tl
    image2 = imutils.resize(image, width=500)
    hsv2 = cv2.cvtColor(image2, cv2.COLOR_BGR2HSV)
    mask2 = cv2.inRange(hsv2, colorLower, colorUpper)
    mask2 = cv2.erode(mask2 ,None, iterations=2)
    mask2 = cv2.dilate(mask2, None, iterations=2)
    cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
    cnts2 = cnts2[0] if imutils.is_cv2() else cnts2[1]
    center = None
    if len(cnts2) > 0:
        c = max(cnts2, key=cv2.contourArea)
        ((xl, yl), radiusl) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radiusl > 10:
            cv2.circle(image2, (int(xl), int(yl)), int(radiusl),
                       (0, 255, 255), 2)
            pl,tl = LeftservoPosition(int(xl), int(yl))
            cv2.circle(image2, center, 5, (0, 0, 255), -1)
    return image2,pl,tl
        
def video_test(camera_list=[1, 3], frames_per_camera=5):
    global pr
    global tr
    global pl
    global tl
    camera = PiCamera()
    res = (640, 480)
    camera.resolution = res
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=res)

    time.sleep(0.1)
     
    # capture frames from the camera
    i = 0
    current_camera_index = 0

    window_name = "Frame"+str(camera_list[current_camera_index])
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        image = frame.array
        if camera_list[current_camera_index] ==  1:
            image1 = RightCamera(image)
            cv2.imshow(window_name, image1)
        else:
            image2 = LeftCamera(image)
            cv2.imshow(window_name,image2)
        setServoAngle(pr,panPinR,pl,panPinL,tr,tiltPinR,tl,tiltPinL)
        key = cv2.waitKey(1) & 0xFF

        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == 27:
            cv2.destroyAllWindows()
            break

        if i == frames_per_camera:
            i = 0
            
        current_camera_index = (current_camera_index + 1) % (len(camera_list))
        set_camera(camera_list[current_camera_index])
            # Brief sleep to prevent visual glitches when cameras are switched 
        time.sleep(0.0001)
        window_name = "Frame"+str(camera_list[current_camera_index])

            
        i += 1

def main():
    set_camera(1)
    # decide which camera should be used for the experiment e.g. 1,3; how many images should be used after the camera is toggled? e.g. 1
    video_test([1,2], 1)

if __name__ == "__main__":
    main()