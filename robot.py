import math

import wpilib
import wpilib.drive

import wpimath
import wpimath.filter
import wpimath.controller

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d, Pose2d

import Components.drivetrain
import Components.vision
import Components.algae_grabber
import Components.elevator

TAG_ORIGIN = Pose2d(-8.774, -4.032, Rotation2d.fromDegrees(0))

TAG_LIST = [
    Pose2d(wpimath.units.inchesToMeters(657.37), wpimath.units.inchesToMeters(25.80), Rotation2d.fromDegrees(126)),
    Pose2d(wpimath.units.inchesToMeters(657.37), wpimath.units.inchesToMeters(291.20), Rotation2d.fromDegrees(234)),
    Pose2d(wpimath.units.inchesToMeters(455.15), wpimath.units.inchesToMeters(317.15), Rotation2d.fromDegrees(270)),
    Pose2d(wpimath.units.inchesToMeters(365.20), wpimath.units.inchesToMeters(241.64), Rotation2d.fromDegrees(0)),
    Pose2d(wpimath.units.inchesToMeters(365.20), wpimath.units.inchesToMeters(75.39), Rotation2d.fromDegrees(0)),
    Pose2d(wpimath.units.inchesToMeters(530.49), wpimath.units.inchesToMeters(130.17), Rotation2d.fromDegrees(300)),
    Pose2d(wpimath.units.inchesToMeters(546.87), wpimath.units.inchesToMeters(158.50), Rotation2d.fromDegrees(0)),
    Pose2d(wpimath.units.inchesToMeters(530.49), wpimath.units.inchesToMeters(186.83), Rotation2d.fromDegrees(60)),
    Pose2d(wpimath.units.inchesToMeters(497.77), wpimath.units.inchesToMeters(186.83), Rotation2d.fromDegrees(120)),
    Pose2d(wpimath.units.inchesToMeters(481.39), wpimath.units.inchesToMeters(158.50), Rotation2d.fromDegrees(180)),
    Pose2d(wpimath.units.inchesToMeters(497.77), wpimath.units.inchesToMeters(130.17), Rotation2d.fromDegrees(240)),
    Pose2d(wpimath.units.inchesToMeters(33.51), wpimath.units.inchesToMeters(25.80), Rotation2d.fromDegrees(54)),
    Pose2d(wpimath.units.inchesToMeters(33.51), wpimath.units.inchesToMeters(291.20), Rotation2d.fromDegrees(306)),
    Pose2d(wpimath.units.inchesToMeters(325.68), wpimath.units.inchesToMeters(241.64), Rotation2d.fromDegrees(180)),
    Pose2d(wpimath.units.inchesToMeters(325.68), wpimath.units.inchesToMeters(75.39), Rotation2d.fromDegrees(180)),
    Pose2d(wpimath.units.inchesToMeters(235.73), wpimath.units.inchesToMeters(-0.15), Rotation2d.fromDegrees(90)),
    Pose2d(wpimath.units.inchesToMeters(160.39), wpimath.units.inchesToMeters(130.17), Rotation2d.fromDegrees(240)),
    Pose2d(wpimath.units.inchesToMeters(144.00), wpimath.units.inchesToMeters(158.50), Rotation2d.fromDegrees(180)),
    Pose2d(wpimath.units.inchesToMeters(160.39), wpimath.units.inchesToMeters(186.83), Rotation2d.fromDegrees(120)),
    Pose2d(wpimath.units.inchesToMeters(193.10), wpimath.units.inchesToMeters(186.83), Rotation2d.fromDegrees(60)),
    Pose2d(wpimath.units.inchesToMeters(209.49), wpimath.units.inchesToMeters(158.50), Rotation2d.fromDegrees(0)),
    Pose2d(wpimath.units.inchesToMeters(193.10), wpimath.units.inchesToMeters(130.17), Rotation2d.fromDegrees(300))
]
# 657.37  25.80 126
# 657.37 291.20 234
# 455.15 317.15 270
# 365.20 241.64   0
# 365.20  75.39   0
# 530.49 130.17 300
# 546.87 158.50   0
# 530.49 186.83  60
# 497.77 186.83 120
# 481.39 158.50 180
# 497.77 130.17 240
#  33.51  25.80  54
#  33.51 291.20 306
# 325.68 241.64 180
# 325.68  75.39 180
# 235.73  -0.15  90
# 160.39 130.17 240
# 144.00 158.50 180
# 160.39 186.83 120
# 193.10 186.83  60
# 209.49 158.50   0
# 193.10 130.17 300


class State():

    def __init__(self, state: str):
        self.state = state
        pass

    def changeState(self, state: str):
        self.state = state

    def getState(self):
        return self.state

class MyRobot(wpilib.TimedRobot):

    def __init__(self) -> None:
        print("\nMyRobot.__init__ executed. Setting up robot...")
        super().__init__()
        print("\n[MyRobot.__init__] Initializing controllers...")
        self.driver1 = wpilib.XboxController(0)
        self.driver2 = wpilib.XboxController(1)
        print("\n[MyRobot.__init__] Initializing drivetrain...")
        self.drivetrain = Components.drivetrain.Drivetrain()
        # self.claw = Components.claw.Claw()
        # self.arm = Components.arm.Arm()
        self.elevator = Components.elevator.Elevator()

        self.algae_grabber = Components.algae_grabber.AlgaeGrabber()

        # self.state = State("disabled")

        # self.xsl = wpimath.filter.SlewRateLimiter(3)  # x speed limiter
        # self.ysl = wpimath.filter.SlewRateLimiter(3)  # y rate limiter
        # self.rl = wpimath.filter.SlewRateLimiter(3)  # rot limiter

        self.position_test = False

        self.repositioning = False

        print("\n[MyRobot.__init__] Setting drivetrain to field-relative mode for teleop...")
        self.field_relative_drive = True

        self.rotation_track_test = Rotation2d(1, 0)

        print("\n[MyRobot.__init__] Setting autonomous_state to 0...")
        self.autonomous_state = 0
        print("\n[MyRobot.__init__] Marking autonomous as not in flight...")
        self.autonomous_in_flight = False  # Make sure we don't accidentally stage immediately after startup
        print("\n[MyRobot.__init__] Setting up autonomous coordinates...")
        self.autonomous_coords = [(-4, -1, Rotation2d(-1, 0)),
                                  (-4,  2, Rotation2d(-1, 0)),
                                  (-2,  2, Rotation2d(-1, 0))]
        print(self.autonomous_coords)

        print("\n[MyRobot.__init__] Initializing vision...")
        self.vision = Components.vision.Vision()

        print("\nMyRobot.__init__ completed.")

    def __del__(self):
        print("\nMyRobot.__del__ executed. Shutting down...")
        self.drivetrain.stop()
        self.drivetrain.disable()
        print("\n[MyRobot.__del__] Drivetrain stopped and disabled.")
        # Other shutdown code here
        print("\nMyRobot.__del__ completed.")

    def disabledInit(self):
        self.drivetrain.stop()
        self.drivetrain.disable()
        self.algae_grabber.disable()
        # self.claw.Disable()
        # self.claw.Stop()
        # self.elevator.Disable()
        # self.elevator.Stop()
        # self.arm.Disable()
        # self.arm.Stop()

    def disabledExit(self):
        self.drivetrain.reset()
        self.drivetrain.enable()
        self.algae_grabber.enable()

    def autonomousInit(self):
        self.drivetrain.set_robot_location(-2, -1, Rotation2d(-1, 0))
        self.autonomous_state = 0

    def autonomousExit(self):
        self.autonomous_in_flight = False
        self.drivetrain.stop()
        self.drivetrain.disable()

    def autonomousPeriodic(self):
        # Make sure we hit the target coordinate
        if self.autonomous_in_flight and not self.drivetrain.arrived_at_target():
            return

        if self.autonomous_state >= len(self.autonomous_coords):
            self.drivetrain.stop()
            if self.autonomous_in_flight:
                print("Done")
            self.autonomous_in_flight = False
            return

        self.autonomous_in_flight = True
        xpos, ypos, heading = self.autonomous_coords[self.autonomous_state]
        self.drivetrain.drive_vector_position(xpos, ypos, heading)
        print(f"Running stage {self.autonomous_state}...")
        self.autonomous_state += 1

    # def robot(self):
    #     pass
    #     # self.robotcontainer = RobotContainer()
    #     # self.drivetrain = self.robotcontainer.drivetrain

    def robotPeriodic(self):
        # try:
        #     self.vision.poll()
        #     self.test_vision()
        # except:
        #     pass
        # self.arm.Update()
        # self.claw.Update()
        # self.elevator.Update()
        try:
            self.drivetrain.update()
        except:
            pass

        self.algae_grabber.update()

    def teleopInit(self):
        self.slow = 4
        self.algae_grabber.zero_arm()
        # self.drivetrain.set_robot_location(-3, 0, Rotation2d(-1, 0))

    def teleopPeriodic(self):
        # self.robotcontainer = RobotContainer()
        self.handle_algae_grabber()
        self.handle_drivetrain()
        self.handleelevator()
        # print(self.drivetrain.odometry.getPose())

    def handle_algae_grabber(self):
        if self.driver1.getPOV() == 180:
            self.algae_grabber.lower_arm()
            self.slow = 1
        elif self.driver1.getPOV() == 0:
            self.algae_grabber.raise_arm()
            self.slow = 4
        elif self.driver1.getPOV() == 270:
            self.algae_grabber.release_algae()
        elif self.driver1.getPOV() == 90:
            self.algae_grabber.grab_algae()
        # else:
            # self.algae_grabber.stop_arm()

    def handleelevator(self):
        # Commented out rightBumber functionality for testing with other purposes

        # if self.driver2.getRightBumper():
        #     self.elevator.CoralEater(0.3)
        if self.driver2.getLeftBumper():
            self.elevator.CoralEater(-0.3)
        else:
            self.elevator.CoralEater(0)

        # if self.driver2.getAButton() and self.elevator.getLimit2() == True:
        #     self.elevator.EleExtend(1)
        #
        # if self.driver2.getBButton() and self.elevator.getLimit3() == True:
        #     self.elevator.EleExtend(1)
        #
        # if self.driver2.getXButton() and self.elevator.getLimit4() == True:
        #     self.elevator.EleExtend(1)
        #
        # if self.driver2.getYButton() and self.elevator.getLimit1() == True:
        #     self.elevator.EleExtend(-1)
        #
        # elif self.driver2.getRightStickButton() and self.driver2.getAButton() and self.elevator.getLimit2() == True:
        #     self.elevator.EleExtend(-1)
        #
        # elif self.driver2.getRightStickButton() and self.driver2.getBButton() and self.elevator.getLimit3() == True:
        #     self.elevator.EleExtend(-1)
        #
        # else:
        #     self.elevator.EleExtend(0)


        # Test functionality with different modes. Requires other testing first, so for now its commented out.

        # if eleMode == 'stick':
        #     self.elevator.EleExtend(self.driver2.getRightY())
        # elif eleMode == 'button':
        #     if self.driver2.getRightBumper():
        #         self.elevator.EleExtend(0.8)

        # Testing the button-triggered EleExtend function. Also modified the function in elevator.py for testing with rotations.

        if self.driver2.getRightBumper():
            self.elevator.EleExtend(0.5)

    def handle_drivetrain(self):
        if self.repositioning and self.drivetrain.arrived_at_target():
            self.repositioning = False
            self.position_test = False
            print("We've arrived!")

        # if self.driver1.getAButtonPressed():
        #     self.position_test = True
        #     self.rotation_track_test = Rotation2d(1, 0)
        # if self.driver1.getBButtonPressed():
        #     self.position_test = False
        #     self.repositioning = False

        if self.driver1.getYButtonPressed():
            # self.drivetrain.set_wheel_angles(Rotation2d(1, 0))
            self.drivetrain.reset()
            return

        if self.driver1.getXButton():
            self.drivetrain.stop()
            self.drivetrain.set_wheel_angles(Rotation2d.fromDegrees(0))
            return

        xspeed = self.driver1.getRightX() * self.slow
        yspeed = self.driver1.getRightY() * self.slow

        # print(xspeed)
        # print(yspeed)

        rot_speed = self.driver1.getLeftX() * math.pi * 2

        if self.position_test:
            # print(self.drivetrain.odometry.getPose())
            if not self.repositioning:
                self.drivetrain.drive_vector_position(0, 0, Rotation2d(1, 0))
                self.repositioning = True
        else:
            if self.field_relative_drive:
                self.drivetrain.drive_vector_velocity_field_relative(-yspeed, -xspeed, -rot_speed)
            else:
                self.drivetrain.drive_vector_velocity(-yspeed, -xspeed, -rot_speed)

    def test_vision(self):
        time_since_data_update, robot_pose_field_space = self.vision.get_position_in_field()
        if time_since_data_update is None:
            return

        if time_since_data_update > 0.08:
            return

        _, target_id, robot_pose_target_space = self.vision.get_robot_position_in_target()

        print(f"[MyRobot.test_vision] Time since last update is {time_since_data_update}s, last target_id = {target_id}")
        print(f"[MyRobot.test_vision] robot_pose_field_space = {robot_pose_field_space}")
        print(f"[MyRobot.test_vision] robot_pose_target_space = {robot_pose_target_space}")
        # print(f"[MyRobot.test_vision] angles are {robot_pose_field_space.rotation().degrees()}, {robot_pose_target_space.rotation().degrees()}")
        print(f"[MyRobot.test_vision] Target pose is {self.get_tag_position(target_id)}")
        print(f"[MyRobot.test_vision] Relative to corner should be {self.get_robot_from_corner(robot_pose_field_space)}")

    def get_robot_from_corner(self, robot_pose: Pose2d) -> Pose2d:
        return robot_pose.relativeTo(TAG_ORIGIN)

    def get_tag_position(self, tag_id: int) -> Pose2d:
        return TAG_LIST[tag_id - 1]
