import cv2
import numpy as np
import requests
from io import BytesIO
import serial
import time

ser = serial.Serial('/dev/ttyACM0',115200)

def transform_and_detect_color(video_url='http://172.24.96.1:56000/mjpeg'):
    # Open the MJPEG video stream
    cap = cv2.VideoCapture(video_url)

    # Check if the video stream opened successfully
    if not cap.isOpened():
        print("Error: Unable to open video stream.")
        return

    try:
        while True:
            # Read a frame from the video stream
            ret, frame = cap.read()

            # Check if the frame is successfully captured
            if not ret:
                print("Error: Unable to capture frame.")
                break

            # Define the Cartesian plane ranges
            height, width, _ = frame.shape
            x_range = np.linspace(-90, 90, width)
            y_range = np.linspace(-90, 90, height)

            # Convert the frame to the HSV color space
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define the range for blue color in HSV
            lower_blue = np.array([100, 50, 50])
            upper_blue = np.array([130, 255, 255])

            # Threshold the frame to get only blue color
            blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

            # Find contours in the blue mask
            contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Initialize coordinates to (0, 0)
            x_cartesian = 0
            y_cartesian = 0

            # Draw a point on the frame at the center of the blue object
            if contours:
                # Get the centroid of the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Calculate Cartesian coordinates based on the center of the blue object
                x_cartesian = round(x_range[cx])
                y_cartesian = round(y_range[cy])

                # Draw a point on the frame
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                x_cartesian_bytes = bytes(str(x_cartesian), 'utf-8')
                y_cartesian_bytes = bytes(str(y_cartesian), 'utf-8')

                # Ensure a consistent length for each value (adjust as needed)
                x_cartesian_bytes = x_cartesian_bytes.ljust(8, b'\0')
                y_cartesian_bytes = y_cartesian_bytes.ljust(8, b'\0')

                ser.write(x_cartesian_bytes)
                ser.write(y_cartesian_bytes)

            # Print the Cartesian coordinates
            print("Blue Object Coordinates (x, y): ({}, {})".format(x_cartesian, y_cartesian))

            # Display the original image
            cv2.imshow("Original", frame)
            time.sleep(0.02)

            # Break the loop if 'Esc' key is pressed
            if cv2.waitKey(1) == 27:
                break

    finally:
        # Release the video stream and close all OpenCV windows
        cap.release()
        cv2.destroyAllWindows()

# Call the function with the specified MJPEG video stream URL
transform_and_detect_color('http://172.24.96.1:56000/mjpeg')
