
import cv2
import imutils
import numpy as np
import threading
import time

import sys
import os
from stitch import stitch

global colorLower
colorLower = (110, 50, 50)
global colorUpper
colorUpper = (130, 255, 255)




def colour(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    center = None
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((xr, yr), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 10:
            cv2.circle(image, (int(xr), int(yr)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(image, center, 5, (0, 0, 255), -1)
    return image


camera = cv2.VideoCapture(0)
ret,image = camera.read()

while ret:
    ret,image = camera.read()
    processed_image = colour(image)
    final = stitch(processed_image,processed_image)
    cv2.imshow('Right -------- Left', final)


    key = cv2.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop
    if key == 27:
        cv2.destroyAllWindows()
        break
