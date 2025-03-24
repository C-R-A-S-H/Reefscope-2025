from photonlibpy import *
import cv2
import numpy as np

global algaeVisible
global algaeYaw

global coralVisable

class autoaim():
    def __init__(self):
        self.alcam = PhotonCamera("alcam")
        self.corcam = PhotonCamera("corcam")

    def aimingal(self): 
        global algaeYaw
        global algaeVisible
        algaeYaw = 0.0
        algaeVisible = False
        results = self.alcam.getAllUnreadResults()
        if len(results) > 0:
            result = results[-1]
            frame = result.getFrame()

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_algae = np.array([30, 40, 40])
            upper_algae = np.array([90, 255, 255])
            
            mask = cv2.inRange(hsv, lower_algae, upper_algae)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                algaeVisible = True
                algaeYaw = self.calculateYaw(x, x + w, frame.shape[1])

    def aimingcor(self):
        global coralVisable

        coralVisable = False
        results = self.corcam.getAllUnreadResults()
        if len(results) > 0:
            result = results[-1]
            frame = result.getFrame()

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_coral = np.array([0,0,200])
            upper_coral = np.array([180,55,255])

            mask = cv2.inRange(hsv, lower_coral, upper_coral)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                coralVisable = True
                coralYaw = self.calculateYaw(x, x + w, frame.shape[1])


    def calculateYaw(self, startX, endX, frameWidth):
        centerX = (startX + endX) / 2.0
        yaw = (centerX - frameWidth / 2.0) * (55.0 / frameWidth)
        return yaw