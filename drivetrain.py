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

        self.blr.setOpenLoopRampRate(0.2)
        self.brr.setOpenLoopRampRate(0.2)
        self.flr.setOpenLoopRampRate(0.2)
        self.frr.setOpenLoopRampRate(0.2)

        self.bld.setOpenLoopRampRate(0.2)
        self.brd.setOpenLoopRampRate(0.2)
        self.fld.setOpenLoopRampRate(0.2)
        self.frd.setOpenLoopRampRate(0.2)

        self.bldenc = self.bld.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.brdenc = self.brd.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.fldenc = self.fld.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.frdenc = self.frd.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)

        self.blenc = ctre.hardware.CANcoder(12)
        self.brenc = ctre.hardware.CANcoder(11)
        self.flenc = ctre.hardware.CANcoder(13)
        self.frenc = ctre.hardware.CANcoder(10)

        self.lcs = ChassisSpeeds(0, 0, 0) #last chassis speed

        rkp = 2
        rki = 0
        rkd = 0.1
        self.blPID = controller.PIDController(rkp, rki, rkd)
        self.blPID.enableContinuousInput(-0.5, 0.5)
        self.blPID.setSetpoint(0.0)
        self.brPID = controller.PIDController(rkp, rki, rkd)
        self.brPID.enableContinuousInput(-0.5, 0.5)
        self.brPID.setSetpoint(0.0)
        self.flPID = controller.PIDController(rkp, rki, rkd)
        self.flPID.enableContinuousInput(-0.5, 0.5)
        self.flPID.setSetpoint(0.0)
        self.frPID = controller.PIDController(rkp, rki, rkd + -0.1)
        self.frPID.enableContinuousInput(-0.5, 0.5)
        self.frPID.setSetpoint(0.0)

        self.rrPID = controller.PIDController(0.001, 0, 0)

        dkp = 0.01
        dki = 0
        dkd = 0

        self.bldPID = controller.ProfiledPIDContoller(dkp, dki, dkd,
                                                      wpimath.trajectory.TrapezoidProfile.Contraints(3, 10))
        self.brdPID = controller.ProfiledPIDController(dkp, dki, dkd,
                                                       wpimath.trajectory.TrapezoidProfile.Contraints(3, 10))
        self.fldPID = controller.ProfiledPIDController(dkp, dki, dkd,
                                                      wpimath.trajectory.TrapezoidProfile.Contraints(3, 10))
        self.frdPID = controller.ProfiledPIDController(dkp, dki, dkd,
                                                       wpimath.trajectory.TrapezoidProfile.Contraints(3, 10))

        self.gyro = navx.AHRS.create_i2c(wpilib.I2C.Port.kMXP)

        self.gyro.enableLogging(True)

        bll = Translation2d(-2.08, -2.08)
        brl = Translation2d(-2.08, 2.08)
        fll = Translation2d(2.08, 2.75)
        frl = Translation2d(2.08, -2.75)

        self.kinematics = SwerveDrive4Kinematics(
            bll, brl, fll, frl
        )