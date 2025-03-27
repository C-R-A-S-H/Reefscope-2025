import phoenix6

import rev

from enum import Enum, auto
import time

ELEVATOR_L_MOTOR = 20
ELEVATOR_R_MOTOR = 19

CORAL_GRABBER_MOTOR = 22
CORAL_GRABBER_SPEED = 0.5
CORAL_GRABBER_RUNTIME = 0.3

ELEVATOR_SOFT_OFFSET = 15

# Define the constraints for elevator positions.
# Soft min/max are the points where the elevator will stop in manual control.
ELEVATOR_MIN = 0
ELEVATOR_SOFT_MIN = ELEVATOR_MIN + ELEVATOR_SOFT_OFFSET
ELEVATOR_MAX = 120  # This is a bit conservative for max, but I don't want to throw a chain
ELEVATOR_SOFT_MAX = ELEVATOR_MAX - ELEVATOR_SOFT_OFFSET

# Original snap code looked like this:

"""
self.snaps = {
    'A': 48,
    'B': 40,
    'Y': 70,
    'X': 115
}  # Assign levels to buttons
"""

# Since we don't care about the specifics of which button snaps where, we
# assign the targets to a bunch of constants instead.

# Targets are in units of rotor rotations.
ELEVATOR_GROUND_TARGET = 0
ELEVATOR_L1_TARGET = 40
ELEVATOR_L2_TARGET = 70
ELEVATOR_L3_TARGET = 115
ELEVATOR_PICKUP_TARGET = 48

# Max elevator velocity, in rotor RPM (I think?), for PositionDutyCycle.
ELEVATOR_VELOCITY = 3


def clamp(value, min_value, max_value):
    """Why does Python not provide clamp of all things???"""
    return max(min_value, min(value, max_value))


class _ElevatorState(Enum):
    """Elevator state machine states. Includes in-motion states so we can stop the motor on arrival."""

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
    """Coral grabber state machine states. These don't do too much right now, gotta see if we can add a sensor or two."""

    GRABBER_IDLE = 0
    GRABBER_INTAKE = auto()
    GRABBER_REJECT = auto()
    GRABBER_IN_MOTION = auto()


class CoralGrabber():
    """Combined coral grabber and elevator class, because call stack optimization or some such nonsense."""

    def __init__(self):
        self.elevator_l_motor = phoenix6.hardware.TalonFX(ELEVATOR_L_MOTOR, "rio")
        self.elevator_r_motor = phoenix6.hardware.TalonFX(ELEVATOR_R_MOTOR, "rio")

        self.coral_grabber_motor = rev.SparkMax(CORAL_GRABBER_MOTOR, rev.SparkLowLevel.MotorType.kBrushless)

        self.disabled = True

        self.elevator_state = _ElevatorState.ELEVATOR_IDLE
        self.grabber_state = _GrabberState.GRABBER_IDLE

        self.grabber_motion_time = time.perf_counter()

        # Elevator zero offsets
        self.elevator_l_offset = 0
        self.elevator_r_offset = 0

        # Elevator min and max values for each motor, pre-offset to save a calculation.
        self.elevator_l_min = ELEVATOR_MIN
        self.elevator_l_soft_min = ELEVATOR_SOFT_MIN
        self.elevator_l_max = ELEVATOR_MAX
        self.elevator_l_soft_max = ELEVATOR_SOFT_MAX

        self.elevator_r_min = ELEVATOR_MIN
        self.elevator_r_soft_min = ELEVATOR_SOFT_MIN
        self.elevator_r_max = ELEVATOR_MAX
        self.elevator_r_soft_max = ELEVATOR_SOFT_MAX

        # Make sure that the offsets are actually correct
        self.reset_elevator_offsets()

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

    def reset_elevator_offsets(self):
        # Get the rotor positions for each motor, assuming we're at zero
        self.elevator_l_offset = self.elevator_l_motor.get_rotor_position().value_as_double
        self.elevator_r_offset = self.elevator_r_motor.get_rotor_position().value_as_double

        # Update the left motor min and max based off of the offset
        self.elevator_l_min = self.elevator_l_offset + ELEVATOR_MIN
        self.elevator_l_soft_min = self.elevator_l_offset + ELEVATOR_SOFT_MIN
        self.elevator_l_max = self.elevator_l_offset + ELEVATOR_MAX
        self.elevator_l_soft_max = self.elevator_l_offset + ELEVATOR_SOFT_MAX

        # As well as the right side
        self.elevator_r_min = self.elevator_r_offset + ELEVATOR_MIN
        self.elevator_r_soft_min = self.elevator_r_offset + ELEVATOR_SOFT_MIN
        self.elevator_r_max = self.elevator_r_offset + ELEVATOR_MAX
        self.elevator_r_soft_max = self.elevator_r_offset + ELEVATOR_SOFT_MAX

    def enable(self):
        # Make sure we start up in a known state.
        self.elevator_state = _ElevatorState.ELEVATOR_IDLE
        self.grabber_state = _GrabberState.GRABBER_IDLE

        # Set the elevator motors to brake on neutral
        self.elevator_l_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.elevator_r_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)

        # Update self.disabled last here so that our state is safe first.
        self.disabled = False

    def disable(self):
        # Update self.disabled first, since we don't want any funny business from the elevator.
        self.disabled = True

        # Enter idle state
        self.elevator_state = _ElevatorState.ELEVATOR_IDLE
        self.grabber_state = _GrabberState.GRABBER_IDLE

        # Just to be safe, disable all of the motors.
        self.elevator_l_motor.disable()
        self.elevator_r_motor.disable()
        self.coral_grabber_motor.disable()

        # Set the neutral mode to coast so the elevator can be reset by hand.
        self.elevator_l_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.COAST)
        self.elevator_r_motor.setNeutralMode(phoenix6.signals.NeutralModeValue.COAST)

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
