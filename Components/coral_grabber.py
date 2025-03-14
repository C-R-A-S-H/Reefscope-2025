import phoenix6

import rev

from enum import Enum, auto
import time

ELEVATOR_L_MOTOR = 20
ELEVATOR_R_MOTOR = 19

CORAL_GRABBER_MOTOR = 18
CORAL_GRABBER_SPEED = 0.5
CORAL_GRABBER_RUNTIME = 0.3

ELEVATOR_MIN = 0
ELEVATOR_MAX = 130

ELEVATOR_GROUND_TARGET = 0
ELEVATOR_L1_TARGET = 30
ELEVATOR_L2_TARGET = 60
ELEVATOR_L3_TARGET = 90
ELEVATOR_PICKUP_TARGET = 40

ELEVATOR_VELOCITY = 3


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

class _ElevatorState(Enum):
    ELEVATOR_IDLE = 0
    ELEVATOR_GROUND = auto()
    ELEVATOR_PICKUP = auto()
    ELEVATOR_L1 = auto()
    ELEVATOR_L2 = auto()
    ELEVATOR_L3 = auto()
    ELEVATOR_MOVING_GROUND = auto()
    ELEVATOR_MOVING_L1 = auto()
    ELEVATOR_MOVING_L2 = auto()
    ELEVATOR_MOVING_L3 = auto()
    ELEVATOR_MOVING_PICKUP = auto()

class _GrabberState(Enum):
    GRABBER_IDLE = 0
    GRABBER_INTAKE = auto()
    GRABBER_REJECT = auto()
    GRABBER_IN_MOTION = auto()

class CoralGrabber():
    def __init__(self):
        self.elevator_l_motor = phoenix6.hardware.TalonFX(ELEVATOR_L_MOTOR, "rio")
        self.elevator_r_motor = phoenix6.hardware.TalonFX(ELEVATOR_R_MOTOR, "rio")

        self.coral_grabber_motor = rev.SparkMax(CORAL_GRABBER_MOTOR, rev.SparkLowLevel.MotorType.kBrushless)

        self.disabled = True

        self.elevator_state = _ElevatorState.ELEVATOR_IDLE
        self.grabber_state = _GrabberState.GRABBER_IDLE

        self.grabber_motion_time = time.perf_counter()

        self.elevator_l_min = ELEVATOR_MIN
        self.elevator_l_offset = 0
        self.elevator_l_max = ELEVATOR_MAX

        self.elevator_r_min = ELEVATOR_MIN
        self.elevator_r_offset = 0
        self.elevator_r_max = ELEVATOR_MAX

        self.zero_elevator()

    def update(self):
        if self.disabled:
            self.elevator_l_motor.disable()
            self.elevator_r_motor.disable()
            self.coral_grabber_motor.disable()
            self.elevator_state = _ElevatorState.ELEVATOR_IDLE
            self.grabber_state = _GrabberState.GRABBER_IDLE
            return
        self._update_elevator()
        self._update_grabber()

    def _update_elevator(self):
        if self.elevator_state == _ElevatorState.ELEVATOR_GROUND:
            self.elevator_state = _ElevatorState.ELEVATOR_MOVING_GROUND
            target_l, target_r = self._get_motor_targets(ELEVATOR_GROUND_TARGET)
            self._set_elevator_targets(target_l, target_r)

        if self.elevator_state == _ElevatorState.ELEVATOR_L1:
            self.elevator_state = _ElevatorState.ELEVATOR_MOVING_L1
            target_l, target_r = self._get_motor_targets(ELEVATOR_L1_TARGET)
            self._set_elevator_targets(target_l, target_r)

        if self.elevator_state == _ElevatorState.ELEVATOR_L2:
            self.elevator_state = _ElevatorState.ELEVATOR_MOVING_L2
            target_l, target_r = self._get_motor_targets(ELEVATOR_L2_TARGET)
            self._set_elevator_targets(target_l, target_r)

        if self.elevator_state == _ElevatorState.ELEVATOR_L3:
            self.elevator_state = _ElevatorState.ELEVATOR_MOVING_L3
            target_l, target_r = self._get_motor_targets(ELEVATOR_L3_TARGET)
            self._set_elevator_targets(target_l, target_r)

        if self.elevator_state == _ElevatorState.ELEVATOR_PICKUP:
            self.elevator_state = _ElevatorState.ELEVATOR_MOVING_PICKUP
            target_l, target_r = self._get_motor_targets(ELEVATOR_PICKUP_TARGET)
            self._set_elevator_targets(target_l, target_r)

        if self.elevator_state == _ElevatorState.ELEVATOR_MOVING_GROUND:
            if self.elevator_l_motor.get_rotor_position().value_as_double < self.elevator_l_min + 3:
                self.elevator_state = _ElevatorState.ELEVATOR_IDLE

        if self.elevator_state == _ElevatorState.ELEVATOR_IDLE:
            self.elevator_l_motor.disable()
            self.elevator_r_motor.disable()

    def _update_grabber(self):
        if self.grabber_state == _GrabberState.GRABBER_INTAKE:
            self.coral_grabber_motor.set(CORAL_GRABBER_SPEED)
            self.grabber_motion_time = time.perf_counter()
            self.grabber_state = _GrabberState.GRABBER_IN_MOTION

        if self.grabber_state == _GrabberState.GRABBER_REJECT:
            self.coral_grabber_motor.set(-CORAL_GRABBER_SPEED)
            self.grabber_motion_time = time.perf_counter()
            self.grabber_state = _GrabberState.GRABBER_IN_MOTION

        if self.grabber_state == _GrabberState.GRABBER_IN_MOTION:
            if time.perf_counter() > self.grabber_motion_time + CORAL_GRABBER_RUNTIME:
                self.coral_grabber_motor.disable()
                self.grabber_state = _GrabberState.GRABBER_IDLE

        if self.grabber_state == _GrabberState.GRABBER_IDLE:
            self.coral_grabber_motor.disable()

    def zero_elevator(self):
        current_position_l = self.elevator_l_motor.get_rotor_position().value_as_double
        current_position_r = self.elevator_r_motor.get_rotor_position().value_as_double

        self.elevator_l_min = current_position_l + ELEVATOR_MIN
        self.elevator_l_offset = current_position_l
        self.elevator_l_max = current_position_l + ELEVATOR_MAX

        self.elevator_r_min = current_position_r + ELEVATOR_MIN
        self.elevator_r_offset = current_position_r
        self.elevator_r_max = current_position_r + ELEVATOR_MAX

    def enable(self):
        self.elevator_state = _ElevatorState.ELEVATOR_IDLE
        self.disabled = False

    def disable(self):
        self.disabled = True
        self.elevator_state = _ElevatorState.ELEVATOR_IDLE
        self.grabber_state = _GrabberState.GRABBER_IDLE
        self.elevator_l_motor.disable()
        self.elevator_r_motor.disable()
        self.coral_grabber_motor.disable()

    def _get_motor_targets(self, target: float) -> (float, float):
        target_l = target + self.elevator_l_offset
        target_r = target + self.elevator_r_offset

        target_l = clamp(target_l, self.elevator_l_min, self.elevator_l_max)
        target_r = clamp(target_r, self.elevator_r_min, self.elevator_r_max)

        return target_l, target_r

    def _set_elevator_targets(self, target_l: float, target_r: float):
        elevator_l_target = phoenix6.controls.PositionDutyCycle(target_l, ELEVATOR_VELOCITY, False)
        elevator_r_target = phoenix6.controls.PositionDutyCycle(target_r, ELEVATOR_VELOCITY, False)

        self.elevator_l_motor.set_control(elevator_l_target)
        self.elevator_r_motor.set_control(elevator_r_target)

    def elevator_l1(self):
        self.elevator_state = _ElevatorState.ELEVATOR_L1

    def elevator_ground(self):
        self.elevator_state = _ElevatorState.ELEVATOR_GROUND
