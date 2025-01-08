import math

import phoenix6 as ctre
import rev
import wpimath
from wpimath import controller
from wpimath import trajectory
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

class SwerveModule:
    def __init__(self, drive_can, turn_can, encoder_can):
        self.dm = rev.CANSparkMax(drive_can, rev.CANSparkMax.MotorType.kBrushless)
        self.dme = self.dm.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)

        self.tm = rev.CANSparkMax(turn_can, rev.CANSparkMax.MotorType.kBrushless)

        self.enc = ctre.hardware.CANcoder(encoder_can)

        self.tkp = 2.5
        self.tkd = 0.5

        self.turnPID = controller.PIDContoller(self.tkp, 0, self.tkd)
        self.turnPID.enableContinuousInput(-0.5, 0.5)
        self.turnPID.setSetpoint(0.0)

        self.dkp = 0.001
        self.dki = 0
        self.dkd = 0

        self.DrivePID = controller.ProfiledPIDController(self.dkp, self.dki, self.dkd,
                                                         wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
    def setSwivelDirection(self):
        pass

    def getRotEncPoseDouble(self):
        return self.encoder.get_absolute_position().value_as_double

    def getRotEncPose(self):
        return self.encodere.get_absolute_position().value

    def getDriveEncPose(self):
        return self.driveMotorEncoder.getPosition()

    def getSwervePose(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.getDriveEncPose(),
            Rotation2d(self.ticks2rad(self.getRotEncPoseDouble()))
        )

    def getAbsoluteEncRad(self) -> float:
        angle = self.getRotPoseDouble()
        angle *= 2 * math.pi
        return angle

    def getDriveVelo(self):
        return self.driveMotorEncoder.getVelocity()

    def getState(self):
        return SwerveModuleState(self.getDriveVelocity(), Rotation2d(self.getRotEncPoseDouble))

    def setDesState(self, state: SwerveModuleState):
        self.state = SwerveModuleState.optimize(state, self.getState().angle)
        self.dm.set(self.state.speed / 3)
        self.turnMotor.set(
            self.turnPID.calculate(self.getAbsolueEncRad(), self.state.angle.radians()))

    def ticks2rad(self, thingie):
        """
        :param thingie: ticks
        :return: hehe
        """
        return (thingie / 0.5) * -math.pi

    def setRotPow(self, power):
        self.tm.set(power)