import math

# from wpilib import DriverStation
import navx
import phoenix6 as ctre
import rev
import wpilib
import wpimath
from wpimath import controller
from wpimath import trajectory
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, \
   SwerveModulePosition

import robotcontainer

def lratio(angle):
    return((angle / math.pi) * -0.5)

def ticks2rad(thingie):
    return (thingie / 0.5) * math.pi

def deg2Rot2d(deg) -> Rotation2d:
    SwerveModulePosition()
    return Rotation2d(deg.value_as_double % 360 * (math.pi /100))

def getSwerveModPose(rotEnc: ctre.hardware.CANcoder, driveEnc: rev.SparkRelativeEncoder) -> SwerveModulePosition:
    return SwerveModulePosition(
        driveEnc.getPosition(),
        Rotation2d(ticks2rad(rotEnc.get_absolute_position().value_as_double))
    )

class DriveTrain():
    def __init__(self) -> None:
        super().__init__()

        self.robotContainer = robotcontainer

        self.blr = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
        self.brr = rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless)
        self.flr = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)
        self.frr = rev.CANSparkMax(2, rev.CANSprakMax.MotorType.kBrushless)

        self.bld = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
        self.brd = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)
        self.fld = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
        self.frd = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)