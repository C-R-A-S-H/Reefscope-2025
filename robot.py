import math
import time

import wpilib
import wpilib.drive

import wpimath
import wpimath.filter
import wpimath.controller

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d, Pose2d, Translation2d

import Components.drivetrain
import Components.vision
import Components.algae_grabber
#import Components.elevator

import Components.coral_grabber

DEFAULT_AUTO = "Default"
LEFT_AUTO = "Left"
RIGHT_AUTO = "Right"

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
        
        print("\n[MyRobot.__init__] Adding chooser...")
        self.chooser = wpilib.SendableChooser()
        self.chooser.setDefaultOption("Default", DEFAULT_AUTO)
        self.chooser.addOption("Left", LEFT_AUTO)
        self.chooser.addOption("Right", RIGHT_AUTO)
        wpilib.SmartDashboard.putData("Auto type", self.chooser)
        
        print("\n[MyRobot.__init__] Initializing controllers...")
        self.driver1 = wpilib.XboxController(0)
        self.driver2 = wpilib.XboxController(1)
        print("\n[MyRobot.__init__] Initializing drivetrain...")
        self.drivetrain = Components.drivetrain.Drivetrain()
        # self.claw = Components.claw.Claw()
        # self.arm = Components.arm.Arm()
        # self.elevator = Components.elevator.Elevator()

        self.coral_grabber = Components.coral_grabber.CoralGrabber()
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
        self.temp_auto = False
        self.realigned = False
        print("\n[MyRobot.__init__] Marking autonomous as not in flight...")
        self.autonomous_in_flight = False  # Make sure we don't accidentally stage immediately after startup
        self.autonomous_state_started = False
        self.auto_last_stage_time = time.perf_counter()
        print("\n[MyRobot.__init__] Setting up autonomous coordinates...")
        self.autonomous_coords = [(-4, -1, Rotation2d(-1, 0)),
                                  (-4,  2, Rotation2d(-1, 0)),
                                  (-2,  2, Rotation2d(-1, 0))]
        print(self.autonomous_coords)

        # print("\n[MyRobot.__init__] Initializing vision...")
        self.vision = Components.vision.Vision()

        self.killmepls = False
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
        # self.elevator.Disable()
        self.coral_grabber.stop()
        self.coral_grabber.disable()
        # self.claw.Disable()
        # self.claw.Stop()
        # self.elevator.Disable()
        # self.elevator.Stop()
        # self.arm.Disable()
        # self.arm.Stop()

    def disabledExit(self):
        self.drivetrain.enable()
        self.algae_grabber.enable()
        # self.elevator.Enable()
        self.coral_grabber.enable()

    def autonomousInit(self):
        self.drivetrain.reset()
        # self.drivetrain.set_robot_location(0, 0, Rotation2d.fromDegrees(0))
        self.realigned = True
        self.autonomous_state = 1
        self.temp_auto = False
        self.autonomous_state_started = False
        self.autonomous_in_flight = False
        self.autonomous_target = Pose2d(0, 0, Rotation2d.fromDegrees(0))
        self.autonomous_target_rotation = Rotation2d.fromDegrees(0)
        self.auto_time = 0
        self.coral_grabber.stop()
        self.coral_grabber.grabber_manual()

        selected_auto = self.chooser.getSelected()
        print(f"Starting {selected_auto}...")
        if selected_auto == LEFT_AUTO:
            self.autonomous_target_rotation = Rotation2d.fromDegrees(60)
            self.autonomous_state = 1
        elif selected_auto == RIGHT_AUTO:
            self.autonomous_target_rotation = Rotation2d.fromDegrees(-60)
            self.autonomous_state = 2
        else:
            self.autonomous_target_rotation = Rotation2d.fromDegrees(-60)
            self.autonomous_state = 0

    def autonomousExit(self):
        self.autonomous_in_flight = False
        self.drivetrain.stop()
        self.drivetrain.disable()

    def autonomousPeriodic(self):
        MAX_STATE = 9
        SHARED_AUTO_STATE = 3
        ELEVATOR_INIT_TIME = 0.3
        ELEVATOR_INIT2_TIME = 0.4
        ELEVATOR_RAISE_TIME = 1.05

        if self.autonomous_state >= MAX_STATE:
            self.drivetrain.stop()
            if self.autonomous_in_flight:
                print("Done")
                self.coral_grabber.stop()
            self.autonomous_in_flight = False
            return

        if not self.autonomous_in_flight:
            print(f"Running stage {self.autonomous_state}...")
            self.autonomous_in_flight = True


        # Default
        if self.autonomous_state == 0:
            if not self.autonomous_state_started:
                self.autonomous_state_started = True
                self.drivetrain.drive_vector_position(-2.2, -1.8, self.autonomous_target_rotation)
            else:
                if self.drivetrain.arrived_at_target():
                    self.autonomous_state = SHARED_AUTO_STATE
                    self.autonomous_state_started = False
                    return

        # Left
        if self.autonomous_state == 1:
            if not self.autonomous_state_started:
                self.autonomous_state_started = True
                self.drivetrain.drive_vector_position(-2.2, 0, self.autonomous_target_rotation)
            else:
                if self.drivetrain.arrived_at_target():
                    self.autonomous_state = SHARED_AUTO_STATE
                    self.autonomous_state_started = False
                    return

        # Right
        if self.autonomous_state == 2:
            if not self.autonomous_state_started:
                self.autonomous_state_started = True
                self.drivetrain.drive_vector_position(-2.2, 0, self.autonomous_target_rotation)
            else:
                if self.drivetrain.arrived_at_target():
                    self.autonomous_state = SHARED_AUTO_STATE
                    self.autonomous_state_started = False
                    return

        # Shared
        if self.autonomous_state == 3:
            if not self.autonomous_state_started:
                self.autonomous_state_started = True
                self.coral_grabber.elevator_l2()
            else:
                if self.coral_grabber.elevator_arrived():
                    self.autonomous_state += 1
                    self.autonomous_state_started = False
                    return

        if self.autonomous_state == 4:
            time_since_update, target_id, robot_pose_target = self.vision.get_target_position_in_robot()
            if time_since_update is not None and time_since_update < 0.08 and target_id != -1:
                self.autonomous_target = robot_pose_target
                self.autonomous_state += 1

        if self.autonomous_state == 5:
            if not self.autonomous_state_started:
                self.autonomous_state_started = True
                self.coral_grabber.elevator_l2()
                self.drivetrain.set_positional_constraints(2, 4)
                self.offset_from_target(self.autonomous_target, Translation2d(0.8, 0), self.autonomous_target_rotation)
            else:
                if self.drivetrain.arrived_at_target() and self.coral_grabber.elevator_arrived():
                    self.autonomous_state += 1
                    self.autonomous_state_started = False
                    return

        if self.autonomous_state == 6:
            time_since_update, target_id, robot_pose_target = self.vision.get_target_position_in_robot()
            if time_since_update is not None and time_since_update < 0.08 and target_id != -1:
                self.autonomous_target = robot_pose_target
                self.autonomous_state += 1

        if self.autonomous_state == 7:
            if not self.autonomous_state_started:
                self.autonomous_state_started = True
                self.coral_grabber.elevator_l2()
                self.drivetrain.set_positional_constraints(0.6, 1.5)
                self.offset_from_target(self.autonomous_target, Translation2d(0.096, -0.15), self.autonomous_target_rotation)
            else:
                if self.drivetrain.arrived_at_target() and self.coral_grabber.elevator_arrived():
                    self.autonomous_state += 1
                    self.autonomous_state_started = False
                    return

        if self.autonomous_state == 8:
            if not self.autonomous_state_started:
                self.autonomous_state_started = True
                self.coral_grabber.grabber_speed(0.2)
                self.auto_time = time.perf_counter()
            else:
                if time.perf_counter() > self.auto_time + 3:
                    self.autonomous_state += 1
                    self.autonomous_state_started = False
                    self.coral_grabber.stop()
                    return

        # if self.autonomous_state == 6:
        #     if not self.autonomous_state_started:
        #         self.autonomous_state_started = True
        #         self.drivetrain.drive_vector_position(-0.7, 0, Rotation2d.fromDegrees(0))
        #     else:
        #         if self.drivetrain.arrived_at_target():
        #             self.autonomous_state += 1
        #             self.autonomous_state_started = False
        

    # def robot(self):
    #     pass
    #     # self.robotcontainer = RobotContainer()
    #     # self.drivetrain = self.robotcontainer.drivetrain

    def robotPeriodic(self):
        try:
            self.vision.poll()
            #self.test_vision()
        except:
            pass
        # self.arm.Update()
        # self.claw.Update()
        # self.elevator.Update()
        try:
            self.drivetrain.update()
        except:
            pass

        self.coral_grabber.update()
        self.algae_grabber.update()

    def teleopInit(self):
        self.slow = 4
        self.turn_speed = 1
        self.algae_grabber.zero_arm()
        self.field_relative_drive = True
        self.coral_grabber.grabber_manual()
        if not self.realigned:
            self.drivetrain.reset()
            self.realigned = True
        # self.drivetrain.set_robot_location(-3, 0, Rotation2d(-1, 0))

    def handle_coral_grabber(self):
        if self.driver2.getAButtonPressed():
            self.coral_grabber.elevator_pickup()

        if self.driver2.getBButtonPressed():
            self.coral_grabber.elevator_l1()

        if self.driver2.getYButtonPressed():
            self.coral_grabber.elevator_l2()

        if self.driver2.getXButtonPressed():
            self.coral_grabber.elevator_l3()

        if self.driver2.getStartButtonPressed():
            self.coral_grabber.elevator_ground()

        if self.driver2.getBackButtonPressed():
            self.coral_grabber.grabber_intake()

        if self.driver2.getLeftStickButtonPressed():
            self.coral_grabber.grabber_manual()

        self.coral_grabber.grabber_speed(self.driver2.getLeftY() * 0.1)

    def teleopPeriodic(self):
        # self.robotcontainer = RobotContainer()
        self.handle_algae_grabber()
        self.handle_drivetrain()
        # self.handleelevator()
        self.handle_coral_grabber()
        self.handle_apriltag_facing()
        # print(self.drivetrain.odometry.getPose())

    def handle_algae_grabber(self):
        if self.driver1.getPOV() == 180:
            self.algae_grabber.lower_arm()
            self.turn_speed = 0.7
            self.slow = 3.5
        elif self.driver1.getPOV() == 0:
            self.algae_grabber.raise_arm()
            self.turn_speed = 1
            self.slow = 4
        elif self.driver2.getRightBumper():
            self.algae_grabber.release_algae()
        elif self.driver2.getLeftBumper():
            self.algae_grabber.grab_algae()
        # else:
            # self.algae_grabber.stop_arm()

    def handleelevator(self):
        # Commented out rightBumber functionality for testing with other purposes

        # if self.driver2.getRightBumper():
        #     self.elevator.CoralEater(0.3)
        self.elevator.set_intake_power(-self.driver2.getLeftY())

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

        # D1 is base, D2 is Ele
        # if self.elevator.maxPOS <= self.elevator.kracken1.get_rotor_position().value_as_double <= self.elevator.startPOS:
        self.elevator.EleExtend(-self.driver2.getRightY())

    def handle_apriltag_facing(self):
        OFFSET_R = -0.15
        OFFSET_L = 0.15
        offset_dist = 0
        if self.driver1.getRightBumperPressed():
            self.killmepls = True
            offset_dist = OFFSET_R
        if self.driver1.getLeftBumperPressed():
            self.killmepls = True
            offset_dist = OFFSET_L
        if self.driver1.getRightBumper() or self.driver1.getLeftBumper():  # Or whichever button you prefer lmao
            # Get vision data
            time_since_update, target_id, robot_pose_target = self.vision.get_target_position_in_robot()

            print(robot_pose_target)
            if time_since_update is not None and time_since_update < 0.08 and target_id != -1 and self.killmepls:
                # Calculate desired angle to face the tag directly
                self.killmepls = False
                self.drivetrain.set_positional_constraints(1, 4)
                self.offset_from_target(robot_pose_target, Translation2d(0.08, offset_dist), Rotation2d.fromDegrees(0), False)
            return True
        return False

    def handle_drivetrain(self):
        if self.repositioning and self.drivetrain.arrived_at_target():
            self.repositioning = False
            self.position_test = False
            print("We've arrived!")

        # Check if we're trying to face an AprilTag
        if self.handle_apriltag_facing():
            return

        if self.driver1.getYButtonPressed():
            self.drivetrain.reset()
            return

        if self.driver1.getXButton():
            self.drivetrain.stop()
            self.drivetrain.set_wheel_angles(Rotation2d.fromDegrees(0))
            return

        if self.driver1.getBackButtonPressed():
            self.field_relative_drive = False
        elif self.driver1.getStartButtonPressed():
            self.field_relative_drive = True

        xspeed = self.driver1.getRightX() * self.slow
        yspeed = self.driver1.getRightY() * self.slow
        rot_speed = self.driver1.getLeftX() * math.pi * 2 * self.turn_speed

        if self.position_test:
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
        # print(f"[MyRobot.test_vision] robot_pose_field_space = {robot_pose_field_space}")
        # print(f"[MyRobot.test_vision] robot_pose_target_space = {robot_pose_target_space}")
        # # print(f"[MyRobot.test_vision] angles are {robot_pose_field_space.rotation().degrees()}, {robot_pose_target_space.rotation().degrees()}")
        # print(f"[MyRobot.test_vision] Target pose is {self.get_tag_position(target_id)}")
        # print(f"[MyRobot.test_vision] Relative to corner should be {self.get_robot_from_corner(robot_pose_field_space)}")

    def get_robot_from_corner(self, robot_pose: Pose2d) -> Pose2d:
        return robot_pose.relativeTo(TAG_ORIGIN)

    def get_tag_position(self, tag_id: int) -> Pose2d:
        return TAG_LIST[tag_id - 1]

    def offset_from_target(self, robot_pose_target: Pose2d, offset: Translation2d, rotation: Rotation2d = None, rotabs: bool = True):
        target_position = -robot_pose_target.translation()
        target_rotation = robot_pose_target.rotation()
        offset = offset.rotateBy(robot_pose_target.rotation())
        target_position = target_position + offset
        if rotation is not None:
            self.drivetrain.drive_vector_position_relative(target_position.X(), -target_position.Y(), rotation, rotabs)
        else:
            self.drivetrain.drive_vector_position_relative(target_position.X(), -target_position.Y(), target_rotation)
