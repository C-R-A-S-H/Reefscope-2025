from photonlibpy import *
import cv2
import numpy as np

global algaeVisible
global algaeYaw

class autoaim():
    def __init__(self):
        self.alcam = PhotonCamera("Logitech,_ink._Webcam_C270")

    def aiming(self): 
        global algaeYaw
        global algaeVisible
        algaeYaw = 0.0
        algaeVisible = False
        results = self.alcam.getAllUnreadResults()
        if len(results) > 0:
            result = results[-1]
            frame = result.getFrame()  # Get the frame from the camera

            # Convert the frame to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define the color range for algae in HSV
            lower_algae = np.array([30, 40, 40])  # Adjust these values based on the color of algae
            upper_algae = np.array([90, 255, 255])  # Adjust these values based on the color of algae

            # Create a mask for the algae color
            mask = cv2.inRange(hsv, lower_algae, upper_algae)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                algaeVisible = True
                algaeYaw = self.calculateYaw(x, x + w, frame.shape[1])

    def calculateYaw(self, startX, endX, frameWidth):
        centerX = (startX + endX) / 2.0
        yaw = (centerX - frameWidth / 2.0) * (55.0 / frameWidth)  # Assuming a 60-degree field of view
        return yaw