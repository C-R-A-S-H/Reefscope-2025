from enum import Enum, auto
import math
import time

import phoenix6

import rev

ARM_MOTOR_ID = 21
LEFT_GRAB_MOTOR_ID = 15
RIGHT_GRAB_MOTOR_ID = 14

ARM_RAISED_THRESHOLD = 1.8
ARM_RAISED_TARGET = 0

ARM_LOWERED_THRESHOLD = 2.55
ARM_LOWERED_TARGET = 3.5

ARM_MIN_TRAVEL_POWER = 0.05
ARM_TRAVEL_POWER = 0.3

ARM_VERTICAL_ANGLE = 1.95

MAX_VEL = 2

def get_arm_power(angle: float) -> float:
    angle -= ARM_VERTICAL_ANGLE
    angle /= 20  # gear ratio of arm
    angle *= math.tau
    coeff = math.sin(angle)
    return (ARM_TRAVEL_POWER * coeff)# + ARM_MIN_TRAVEL_POWER

class _ArmState(Enum):
    ARM_IDLE = 0
    ARM_RAISE = auto()
    ARM_RAISING = auto()
    ARM_LOWER = auto()
    ARM_LOWERING = auto()
    ARM_GRAB = auto()
    ARM_GRABBING = auto()
    ARM_RELEASING = auto()
    ARM_RELEASE = auto()


class AlgaeGrabber:
    """Algae grabber mechanism class."""

    def __init__(self):
        self.arm_motor = phoenix6.hardware.TalonFX(ARM_MOTOR_ID, "rio")
        self.arm_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)

        self.zero_arm()

        self.left_grab_motor = rev.SparkMax(LEFT_GRAB_MOTOR_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_grab_motor = rev.SparkMax(RIGHT_GRAB_MOTOR_ID, rev.SparkLowLevel.MotorType.kBrushless)

        self.disabled = True

        self.arm_grab_time = time.monotonic()

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
            request = phoenix6.controls.PositionDutyCycle(self.arm_lowered_target, MAX_VEL, False)
            self.arm_motor.set_control(request)
            self.arm_state = _ArmState.ARM_LOWERING

        if self.arm_state == _ArmState.ARM_LOWERING:
            if self.arm_motor.get_rotor_position().value_as_double > self.arm_lowered_thresh:
                self.arm_state = _ArmState.ARM_IDLE

        if self.arm_state == _ArmState.ARM_RAISE:
            request = phoenix6.controls.PositionDutyCycle(self.arm_raised_target, MAX_VEL, False)
            self.arm_motor.set_control(request)
            self.arm_state = _ArmState.ARM_RAISING

        if self.arm_state == _ArmState.ARM_RAISING:
            if self.arm_motor.get_rotor_position().value_as_double < self.arm_raised_thresh:
                self.arm_state = _ArmState.ARM_IDLE

        if self.arm_state == _ArmState.ARM_GRAB:
            if self.arm_motor.get_rotor_position().value_as_double > self.arm_vert:
                self.arm_state = _ArmState.ARM_GRABBING
                self.arm_grab_time = time.monotonic()
                self.left_grab_motor.set(-0.6)
                self.right_grab_motor.set(-0.6)
                self.arm_motor.disable()
            else:
                self.arm_state = _ArmState.ARM_IDLE

        if self.arm_state == _ArmState.ARM_RELEASE:
            if self.arm_motor.get_rotor_position().value_as_double > self.arm_vert:
                self.arm_state = _ArmState.ARM_RELEASING
                self.arm_grab_time = time.monotonic()
                self.left_grab_motor.set(0.55)
                self.right_grab_motor.set(0.55)
                self.arm_motor.disable()
            else:
                self.arm_state = _ArmState.ARM_IDLE

        if self.arm_state == _ArmState.ARM_GRABBING:
            if time.monotonic() - self.arm_grab_time > 1.5:
                self.arm_state = _ArmState.ARM_IDLE
        elif self.arm_state == _ArmState.ARM_RELEASING:
            if time.monotonic() - self.arm_grab_time > 2.5:
                self.arm_state = _ArmState.ARM_IDLE
        else:
            self.left_grab_motor.disable()
            self.right_grab_motor.disable()

        if self.arm_state == _ArmState.ARM_IDLE:
            # Motor idle code here
            self.arm_motor.disable()
            self.left_grab_motor.disable()
            self.right_grab_motor.disable()

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

    def zero_arm(self):
        current_rot = self.arm_motor.get_rotor_position().value_as_double
        self.arm_raised_thresh = ARM_RAISED_THRESHOLD + current_rot
        self.arm_raised_target = ARM_RAISED_TARGET + current_rot
        self.arm_lowered_thresh = ARM_LOWERED_THRESHOLD + current_rot
        self.arm_lowered_target = ARM_LOWERED_TARGET + current_rot
        self.arm_vert = ARM_VERTICAL_ANGLE + current_rot
