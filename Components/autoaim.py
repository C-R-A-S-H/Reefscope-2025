from photonlibpy import *

global targetVisable
global targetYaw

class autoaim():
    def __init__(self):
        self.alcam = PhotonCamera("Logitech,_ink._Webcam_C270")

    def aiming(self): 
        targetYaw = 0.0
        targetVisable = False
        results = self.alcam.getAllUnreadResults()
        if len(results) > 0:
            result = results[-1]
            for target in result.getTargets():
                if target.getFiducialId() == 1:
                    targetVisable = True
                    targetYaw = target.getYaw()