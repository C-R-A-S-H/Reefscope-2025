from enum import Enum, auto
import math

import phoenix6

import rev

ARM_MOTOR_ID = 21
LEFT_GRAB_MOTOR_ID = 14
RIGHT_GRAB_MOTOR_ID = 15

ARM_RAISED_THRESHOLD = 0.8

ARM_LOWERED_THRESHOLD = 1.5

ARM_MIN_TRAVEL_POWER = 0.05
ARM_TRAVEL_POWER = 0.3

ARM_VERTICAL_ANGLE = 1.2

def get_arm_power(angle: float) -> float:
    angle -= ARM_VERTICAL_ANGLE
    angle /= 20  # gear ratio of arm
    angle *= math.tau
    coeff = math.sin(angle)
    return (ARM_TRAVEL_POWER * coeff)# + ARM_MIN_TRAVEL_POWER

class _ArmState(Enum):
    ARM_IDLE = 0
    ARM_RAISE = auto()
    ARM_LOWER = auto()
    ARM_GRAB = auto()
    ARM_RELEASE = auto()


class AlgaeGrabber:
    """Algae grabber mechanism class."""

    def __init__(self):
        self.arm_motor = phoenix6.hardware.TalonFX(ARM_MOTOR_ID, "rio")
        self.arm_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)

        # self.left_grab_motor = rev.SparkMax(LEFT_GRAB_MOTOR_ID, rev.SparkLowLevel.MotorType.kBrushless)
        # self.right_grab_motor = rev.SparkMax(RIGHT_GRAB_MOTOR_ID, rev.SparkLowLevel.MotorType.kBrushless)

        self.disabled = True

        self.arm_state = _ArmState.ARM_IDLE

    def update(self):
        # Make sure we don't run any arm stuff at idle
        if self.disabled:
            self.arm_motor.disable()
            # self.arm_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
            # self.left_grab_motor.disable()
            # self.right_grab_motor.disable()
            self.arm_state = _ArmState.ARM_IDLE
            return

        if self.arm_state == _ArmState.ARM_LOWER:
            if self.arm_motor.get_rotor_position().value_as_double > ARM_LOWERED_THRESHOLD:
                self.arm_state = _ArmState.ARM_IDLE
            else:
                self.arm_motor.set(-get_arm_power(self.arm_motor.get_rotor_position().value_as_double))

        if self.arm_state == _ArmState.ARM_RAISE:
            if self.arm_motor.get_rotor_position().value_as_double < ARM_RAISED_THRESHOLD:
                self.arm_state = _ArmState.ARM_IDLE
            else:
                self.arm_motor.set(-get_arm_power(self.arm_motor.get_rotor_position().value_as_double))

        if self.arm_state == _ArmState.ARM_GRAB:
            pass

        if self.arm_state == _ArmState.ARM_RELEASE:
            pass

        if self.arm_state == _ArmState.ARM_IDLE:
            # Motor idle code here
            self.arm_motor.disable()

    def enable(self):
        print("[AlgaeGrabber] Entering enabled state")
        self.disabled = False
        self.arm_state = _ArmState.ARM_IDLE

    def disable(self):
        print("[AlgaeGrabber] Entering disabled state")
        self.disabled = True
        self.arm_state = _ArmState.ARM_IDLE

    def raise_arm(self):
        print("[AlgaeGrabber] self.arm_state is set to _ArmState.ARM_RAISE")
        self.arm_state = _ArmState.ARM_RAISE

    def lower_arm(self):
        print("[AlgaeGrabber] self.arm_state is set to _ArmState.ARM_LOWER")
        self.arm_state = _ArmState.ARM_LOWER

    def stop_arm(self):
        print("[AlgaeGrabber] self.arm_state is set to _ArmState.ARM_IDLE")
        self.arm_state = _ArmState.ARM_IDLE

    def grab_algae(self):
        print("[AlgaeGrabber] self.arm_state is set to _ArmState.ARM_GRAB")
        self.arm_state = _ArmState.ARM_GRAB

    def release_algae(self):
        print("[AlgaeGrabber] self.arm_state is set to _ArmState.ARM_RELEASE")
        self.arm_state = _ArmState.ARM_RELEASE
