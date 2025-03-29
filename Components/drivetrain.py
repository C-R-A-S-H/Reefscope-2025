import math
from enum import auto, Enum

# import navx
# # from wpilib import DriverStation
# import phoenix6 as ctre
# import rev
# import wpilib

from navx import AHRS

import wpimath
from wpimath import trajectory
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, \
    SwerveModulePosition

from .swervemodule import SwerveModule

MAX_AUTO_VEL = 4
MIN_AUTO_VEL = 0.5
MAX_AUTO_ACCEL = 8
MIN_AUTO_ACCEL = 0.5

class DrivetrainControlMode(Enum):
    STOP = 0
    VELOCITY_CONTROL = auto()
    POSITION_CONTROL = auto()

def clamp(value, lower_bound, upper_bound):
    return min(max(value, lower_bound), upper_bound)

# Drive motor order is front right, back right, front left, back left
# Because FRC uses +X as forwards and -Y as right for SOME REASON

class Drivetrain():

    def __init__(self):
        self.front_left = SwerveModule(6, 3, 13)
        self.front_right = SwerveModule(5, 2, 10)
        self.back_left = SwerveModule(7, 1, 12)
        self.back_right = SwerveModule(4, 8, 11)

        self.bll_pos = Translation2d(.25, .35)
        self.brl_pos = Translation2d(-.25, .35)
        self.fll_pos = Translation2d(.25, -.35)
        self.frl_pos = Translation2d(-.25, -.35)

        self.kinematics = SwerveDrive4Kinematics(self.bll_pos, self.fll_pos, self.brl_pos, self.frl_pos)
        self.odometry = SwerveDrive4Odometry(self.kinematics, Rotation2d(1, 0),
                                             self.get_swerve_module_positions(),
                                             Pose2d(0, 0, 0))

        self.gyro = AHRS(AHRS.NavXComType.kMXP_SPI, 50)

        self.gyro.enableLogging(True)
        self.gyro.reset()

        self.current_mode = DrivetrainControlMode.VELOCITY_CONTROL

        self.target_pose = Pose2d(0, 0, 0)

        self.target_velocity: ChassisSpeeds = ChassisSpeeds(0, 0, 0)

        self.translation_pid_constraints = trajectory.TrapezoidProfile.Constraints(2, 8)
        self.translation_pid_kP = 4
        self.translation_pid_kI = 1
        self.translation_pid_kD = 0.3
        self.translation_pid_x = wpimath.controller.ProfiledPIDController(self.translation_pid_kP,
                                                                          self.translation_pid_kI,
                                                                          self.translation_pid_kD,
                                                                          self.translation_pid_constraints)
        self.translation_pid_y = wpimath.controller.ProfiledPIDController(self.translation_pid_kP,
                                                                          self.translation_pid_kI,
                                                                          self.translation_pid_kD,
                                                                          self.translation_pid_constraints)
        self.translation_pid_x.disableContinuousInput()
        self.translation_pid_x.setTolerance(0.05)
        self.translation_pid_x.setIZone(0.5)
        self.translation_pid_x.setIntegratorRange(-0.5, 0.5)
        self.translation_pid_y.disableContinuousInput()
        self.translation_pid_y.setTolerance(0.05)
        self.translation_pid_y.setIZone(0.5)
        self.translation_pid_y.setIntegratorRange(-0.5, 0.5)

        self.rotation_pid_constraints = trajectory.TrapezoidProfile.Constraints(math.pi, math.tau)
        self.rotation_pid = wpimath.controller.ProfiledPIDController(7, 1, 0.2,
                                                                     self.rotation_pid_constraints)
        self.rotation_pid.enableContinuousInput(-math.pi, math.pi)
        self.rotation_pid.setTolerance(0.02)
        self.rotation_pid.setIZone(0.3)
        self.rotation_pid.setIntegratorRange(-0.5, 0.5)

        # Reset all of the drive motor encoders
        self.reset_odometry()

        print("End of init")

    def _update_velocity_mode(self):
        pass

    def _update_position_mode(self):
        # print(f"target pose: {self.target_pose.X()}, {self.target_pose.Y()}, {self.target_pose.rotation()}")

        x_vel = self.translation_pid_x.calculate(self.odometry.getPose().X(), self.target_pose.X(), self.translation_pid_constraints)
        if self.translation_pid_x.atGoal():
            x_vel = 0

        y_vel = self.translation_pid_y.calculate(self.odometry.getPose().Y(), self.target_pose.Y(), self.translation_pid_constraints)
        if self.translation_pid_y.atGoal():
            y_vel = 0

        rot_rate = self.rotation_pid.calculate(self.odometry.getPose().rotation().radians(), self.target_pose.rotation().radians())
        if self.rotation_pid.atGoal():
            rot_rate = 0

        # print(f"corrective action: {x_vel}, {y_vel}, {rot_rate}")

        swerve_module_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ChassisSpeeds(x_vel, y_vel, rot_rate), self.get_yaw())
        self.set_swerve_module_states(swerve_module_speeds)

    def update(self):
        self.odometry.update(self.gyro.getRotation2d(), self.get_swerve_module_positions())

        if self.current_mode == DrivetrainControlMode.STOP:
            self.front_left.stop()
            self.front_right.stop()
            self.back_left.stop()
            self.back_right.stop()
            return
        if self.current_mode == DrivetrainControlMode.VELOCITY_CONTROL:
            self._update_velocity_mode()
        if self.current_mode == DrivetrainControlMode.POSITION_CONTROL:
            self._update_position_mode()

        self.front_left.update()
        self.front_right.update()
        self.back_left.update()
        self.back_right.update()

        # test_position = self.front_right.get_position()
        # print(f"front right position: {test_position.distance}, {test_position.angle.degrees()}")
        # print(f"pose: {current_pose.X()}, {current_pose.Y()}, {current_pose.rotation().degrees()}")
        # test_position = self.front_right.get_position()
        # print(f"{test_position.distance}, {test_position.angle.degrees()}")

        # print(self.gyro.getYaw())

    def get_yaw(self) -> Rotation2d:
        # return self.gyro.getRotation2d()
        return self.odometry.getPose().rotation()

    def get_swerve_module_positions(self) -> (SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition):
        return (self.back_left.get_position(), self.front_left.get_position(),
                self.back_right.get_position(), self.front_right.get_position())

    def set_wheel_angles(self, angle):
        self.front_left.set_state_from_speed_and_angle(0, angle)
        self.front_right.set_state_from_speed_and_angle(0, angle)
        self.back_left.set_state_from_speed_and_angle(0, angle)
        self.back_right.set_state_from_speed_and_angle(0, angle)

    def set_wheel_speed_and_angle(self, speed, angle):
        self.front_left.set_state_from_speed_and_angle(speed, angle)
        self.front_right.set_state_from_speed_and_angle(speed, angle)
        self.back_left.set_state_from_speed_and_angle(speed, angle)
        self.back_right.set_state_from_speed_and_angle(speed, angle)

    def set_swerve_module_states(self, speeds: ChassisSpeeds) -> None:
        bl_state, fl_state, br_state, fr_state = self.kinematics.toSwerveModuleStates(speeds, Translation2d(0, 0))

        self.front_left.set_state(fl_state)
        self.front_right.set_state(fr_state)
        self.back_left.set_state(bl_state)
        self.back_right.set_state(br_state)

    def drive_vector_velocity(self, xvel: float, yvel: float, rotrate: float) -> None:
        self.current_mode = DrivetrainControlMode.VELOCITY_CONTROL
        self.set_swerve_module_states(ChassisSpeeds(xvel, yvel, rotrate))

    def drive_vector_velocity_field_relative(self, xvel_abs: float, yvel_abs: float, rotrate: float) -> None:
        self.current_mode = DrivetrainControlMode.VELOCITY_CONTROL
        self.set_swerve_module_states(ChassisSpeeds.fromFieldRelativeSpeeds(ChassisSpeeds(xvel_abs, yvel_abs, rotrate),
                                                                            self.get_yaw()))

    def drive_vector_position(self, xpos: float, ypos: float, rot: Rotation2d) -> None:
        self.target_pose = Pose2d(xpos, ypos, rot.radians())
        self.reset_pids()
        self.current_mode = DrivetrainControlMode.POSITION_CONTROL

    def drive_vector_position_relative(self, xrel: float, yrel: float, rot: Rotation2d) -> None:
        pose = self.odometry.getPose()
        self.drive_vector_position(pose.X() + xrel, pose.Y() + yrel, pose.rotation().rotateBy(rot))

    def arrived_at_target(self) -> bool:
        if self.current_mode != DrivetrainControlMode.POSITION_CONTROL:
            return False
        return self.translation_pid_x.atGoal() and self.translation_pid_y.atGoal() and self.rotation_pid.atGoal()

    def stop(self):
        self.drive_vector_velocity(0, 0, 0)

    def disable(self):
        self.front_left.disable()
        self.front_right.disable()
        self.back_left.disable()
        self.back_right.disable()

    def enable(self):
        self.front_left.enable()
        self.front_right.enable()
        self.back_left.enable()
        self.back_right.enable()

    def reset_gyro(self):
        self.gyro.zeroYaw()

    def reset_odometry(self, xpos: float = 0, ypos: float = 0, heading: Rotation2d = Rotation2d(1, 0)):
        self.front_left.reset_distance()
        self.front_right.reset_distance()
        self.back_left.reset_distance()
        self.back_right.reset_distance()

        self.odometry.resetPosition(Rotation2d.fromDegrees(self.gyro.getYaw()), self.get_swerve_module_positions(), Pose2d(xpos, ypos, heading.radians()))

    def reset(self):
        self.reset_gyro()
        self.reset_odometry()

    def reset_pids(self):
        self.translation_pid_x.reset(self.odometry.getPose().X())
        self.translation_pid_y.reset(self.odometry.getPose().Y())
        self.rotation_pid.reset(self.get_yaw().radians())

    def set_robot_location(self, xpos: float, ypos: float, heading: Rotation2d) -> None:
        self.reset_odometry(xpos, ypos, heading)
        self.reset_pids()
        print(self.odometry.getPose())

    def set_positional_constraints(self, vel: float, accel: float) -> None:
        vel = clamp(vel, MIN_AUTO_VEL, MAX_AUTO_VEL)
        accel = clamp(accel, MIN_AUTO_ACCEL, MAX_AUTO_ACCEL)
        self.translation_pid_constraints = trajectory.TrapezoidProfile.Constraints(vel, accel)

    def get_absolute_position(self) -> Pose2d:
        return self.odometry.getPose()
