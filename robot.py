import wpilib
import wpilib.drive
import wpilib.drive
import wpilib.shuffleboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds

from robotcontainer import RobotContainer

class State():
    def __init__(self, state: str):
        self.state = State
        pass

    def cs(self, state: str):
        self.state = state

    def gs(self):
        return self.state

class MyRobot(wpilib.TimedRobot):

    def __init__(self):
        super().__init__()
        self.driver1 = wpilib.Joystick(0)
        self.driver2 = wpilib.Joystick(1)
        # self.joystickPS5 = wpilib.PS5Controller(0)
        self.toggle = True
        self.amp_toggle = False

        self.state = State('Disabled')

        def disabledPeriodic(self):
            # self.drivetrain.gyro.zeroYaw()
            pass

        def robotInit(self):
            """
            This function is called upon program startup and
            should be used for any initialization code.
            """
            # self.camera1 = photonlibpy.photonCamera.PhotonCamera("Camera1")

            self.robotContainer = RobotContainer()
            self.drivetrain = self.robotContainer.drivetrain
            self.drivetrain.gyro.zeroYaw()

        def teleopInit(self):
            """This function is called once each time the robot enters teleoperated mode."""

            # self.drivetrain.gyro.zeroYaw()  # remove this after testing
            self.slow = 1

        def teleopPeriodic(self):
            self.aimNum = self.autoAim.Aim()
            """This function is called periodically during teleoperated mode."""
            xspeed = self.driver1.getX()
            yspeed = self.driver1.getY()

            if self.driver1.getRawButtonPressed(2):
                self.drivetrain.gyro.zeroYaw()

            if self.driver1.getTrigger():
                tspeed = self.driver1.getZ()
            else:
                tspeed = 0  # self.drivetrain.align()

            if abs(xspeed) < .15:  # applies  a deadzone to the joystick
                xspeed = 0
            if abs(yspeed) < .2:
                yspeed = 0

            if xspeed == 0 and yspeed == 0 and tspeed == 0:  # if no speed is given to the motors there will be no power in any of the motors
                self.drivetrain.frontLeftDrive.set(0)
                self.drivetrain.backRightDrive.set(0)
                self.drivetrain.backLeftDrive.set(0)
                self.drivetrain.frontRightDrive.set(0)

                self.drivetrain.backLeftRotation.set(0)
                self.drivetrain.backRightRotation.set(0)
                self.drivetrain.frontLeftRotation.set(0)
                self.drivetrain.frontRightRotation.set(0)

            # speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-yspeed * 0.5, xspeed * 0.5, tspeed * 0.5, Rotation2d(0.0))

            speeds = ChassisSpeeds.fromRobotRelativeSpeeds(yspeed * self.slow, -xspeed * self.slow, tspeed * 0.8,
                                                           Rotation2d().fromDegrees(
                                                               self.yaw))  # calculates power given to the motors depending on the user inputs
            self.drivetrain.driveFromChassisSpeeds(speeds)
            # self.drivetrain.newDrive(speeds)

            self.drivetrain.driveFromChassisSpeeds(speeds)
            # self.drivetrain.newDrive(speeds)

            # print(self.drivetrain.odometry.getPose())

            print(self.drivetrain.odometry.getPose())

            # self.armPos = self.driver2.getY() * 50

            # if abs(self.armPos) < .10:
            # self.armPos = 0

            if self.driver2.getRawButtonPressed(10):
                self.armPos = 64.66  # on starting intake moves to intake pos
                pass

            self.arm.moveToEncoderPos(self.armPos)

            # self.arm.moveToPosition(self.armPos)
