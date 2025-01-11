import wpilib
import wpilib.drive
import wpilib.shuffleboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds

from robotcontainer import RobotContainer

class State():
    def __init__(self, state: str):
        self.state = state
        pass

    def cs(self, state: str):
        self.state = state

    def gs(self):
        return self.state

class MyRobot(wpilib.TimedRobot):

    def __init__(self):
        super().__init__()
        self.driver1 = wpilib.joystick(0)
        self.driver2 = wpilib.Joystick(1)
        self.toggle = True
        self.amp_toggle = False

        self.state = State('Disabled')

    def disabledPeriodic(self):
        pass

    def robotInit(self):
        self.robotContainer = RobotContainer()
        self.drivetrain = self.robotContainer.drivetrain

        self.drivetrain.gyro.zeroYaw()

    def autonomousInit(self):
        self.Time = wpilib.Timer()
        self.Time.start()

    def autonomousPeriodic(self):
        speeds = self.auto.timeToSpeeds(self.Time.get(), self.drivetrain.odometry.getPose(),
                                        Rotation2d().fromDegrees(-self.yaw).degrees())
        self.drivetrain.driveFromChassisSpeeds(speeds)
        print(f"Time: {self.Time.get}, Odo Pose: {self.drivetrain.odo.getPose()}, Heading: {Rotation2d().fromDegrees(self.yaw)}")

    def teleopInit(self):
        self.slow = 1


    def teleopPeriodic(self):
        xspeed = self.driver1.getX()
        yspeed = self.driver1.getY()

        if self.driver1.getRawButtonPressed(2):
            self.drivetrain.gyro.zeroYaw()

        if self.driver1.getTrigger():
            tspeed = self.driver1.getZ()

        else:
            tspeed = 0

        if xspeed == 0 and yspeed == 0 and tspeed == 0:
            self.drivetrain.bld.set(0)
            self.drivetrain.brd.set(0)
            self.drivetrain.fld.set(0)
            self.drivetrain.frd.set(0)

            self.drivetrain.blr.set(0)
            self.drivetrain.brr.set(0)
            self.drivetrain.flr.set(0)
            self.drivetrain.frr.set(0)

        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(yspeed * self.slow, -yspeed * self.slow, tspeed * 0.8,
                                                       Rotation2d().fromDegrees(
                                                           self.yaw))
        self.drivetrain.driveFromChassisSpeeds(speeds)

        print(self.drivetrain.odo.getPose())

        if __name__ == "__main__":
            wpilib.run(MyRobot)