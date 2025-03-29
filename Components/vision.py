import wpimath
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d, Rotation3d

import limelightresults
import limelightresults
import limelight
import time
import json

import sys

from enum import Enum, auto

LIMELIGHT_RES_X = 640
LIMELIGHT_RES_Y = 480

UNRELIABLE_THRESHOLD_X = LIMELIGHT_RES_X / 6
UNRELIABLE_THRESHOLD_Y = LIMELIGHT_RES_Y / 6

APRILTAG_PIPELINE = 0
ALGAE_PIPELINE = 1

class _VisionState(Enum):
    VISION_DISABLED = 0
    VISION_APRILTAG = auto()
    VISION_ALGAE = auto()


class Vision:
    """Main vision class, responsible for close-in navigation and global positioning with AprilTags"""

    def __init__(self):

        print("\nVision.__init__ executed, initializing vision class...")

        self.vision_state = _VisionState.VISION_DISABLED
        self.good = False
        self.discovered_limelights = limelight.discover_limelights(debug=True, timeout=1)

        print("[Vision.__init__] discovered limelights:", self.discovered_limelights)

        self.ll = None

        self.last_update_time = 0
        self.estimated_position = Pose2d(0, 0, 0)
        self.current_target_id = -1
        self.target_relative_estimated_position = Pose2d(0, 0, 0)
        self.robot_in_target_estimated_position = Pose2d(0, 0, 0)

        self.algae_last_update_time = 0
        self.algae_present = False
        self.algae_angle = Rotation2d.fromDegrees(0)

        if self.discovered_limelights:
            print("[Vision.__init__] Limelights discovered, opening first one...")
            self.limelight_address = self.discovered_limelights[0]
            self.ll = limelight.Limelight(self.limelight_address)
            if self.ll is None:
                print("[Vision.__init__] Limelight object is None!")
                return
            timestamp = time.perf_counter_ns()
            self.ll.enable_websocket()
            timestamp_end = time.perf_counter_ns()
            timestamp_end -= timestamp
            print(f"[Vision.__init__] Took {timestamp_end}ns to enable websocket")
            print("[Vision.__init__] Flagging vision class as good, current limelight = " + self.limelight_address)
            self.good = True
            self.vision_state = _VisionState.VISION_APRILTAG
            self.ll.pipeline_switch(APRILTAG_PIPELINE)
        else:
            print("[Vision.__init__] No limelights found.")

        print("Vision.__init__ completed.")

    def __del__(self):
        print("\nRunning Vision.__del__, destructing...")
        if self.good:
            self.good = False
            print("[Vision.__del__] Vision flagged as good, closing...")
            self.ll.disable_websocket()
        else:
            print("[Vision.__del__] Vision not flagged as good, nothing to do.")

        print("Vision.__del__ completed.")

    def _update_apriltag_data(self, parsed_results: limelightresults.GeneralResult):
        fiducial_results = parsed_results.fiducialResults
        if len(fiducial_results) == 0:
            return
        self.last_update_time = time.monotonic()
        for i in fiducial_results:
            # if i.target_x_pixels < UNRELIABLE_THRESHOLD_X and i.target_y_pixels < UNRELIABLE_THRESHOLD_Y:
            #     continue
            robot_pose_field_space = i.robot_pose_field_space
            target_pose_robot_space = i.target_pose_robot_space
            robot_pose_target_space = i.robot_pose_target_space
            self.estimated_position = Pose2d(robot_pose_field_space[0],
                                             robot_pose_field_space[1],
                                             wpimath.units.degreesToRadians(robot_pose_field_space[5] + 180))
            self.target_relative_estimated_position = Pose3d(target_pose_robot_space[2],
                                                             target_pose_robot_space[0],
                                                             target_pose_robot_space[1],
                                                             Rotation3d(wpimath.units.degreesToRadians(target_pose_robot_space[3]),
                                                                        wpimath.units.degreesToRadians(target_pose_robot_space[4]),
                                                                        wpimath.units.degreesToRadians(target_pose_robot_space[5]))).toPose2d()
            self.robot_in_target_estimated_position = Pose3d(robot_pose_target_space[2],
                                                             robot_pose_target_space[0],
                                                             robot_pose_target_space[1],
                                                             Rotation3d(wpimath.units.degreesToRadians(robot_pose_target_space[3]),
                                                                        wpimath.units.degreesToRadians(robot_pose_target_space[4]),
                                                                        wpimath.units.degreesToRadians(robot_pose_target_space[5]))).toPose2d()
            self.current_target_id = i.fiducial_id

    def _update_algae_data(self, parsed_results: limelightresults.GeneralResult):
        algae_detections = parsed_results.detectorResults
        self.algae_angle = Rotation2d.fromDegrees(algae_detections.target_x_degrees)

    def poll(self):
        if not self.good or self.vision_state == _VisionState.VISION_DISABLED:
            return

        # self.cycle += 1

        # if self.cycle < 10:
            # return

        # self.cycle = 0

        results = self.ll.get_latest_results()

        parsed_results = limelightresults.parse_results(results)

        if parsed_results is not None:
            if self.vision_state == _VisionState.VISION_APRILTAG:
                self._update_apriltag_data(parsed_results)
            if self.vision_state == _VisionState.VISION_ALGAE:
                self._update_algae_data(parsed_results)

    def get_position_in_field(self) -> (wpimath.units.seconds, Pose2d):
        """Returns time in seconds since last update and an estimated pose in field space"""
        if not self.good:
            return None, None

        return (time.monotonic() - self.last_update_time, self.estimated_position)

    def get_target_position_in_robot(self) -> (wpimath.units.nanoseconds, int, Pose2d):
        if not self.good:
            return None, -1, None

        return (time.monotonic() - self.last_update_time, self.current_target_id, self.target_relative_estimated_position)

    def get_robot_position_in_target(self) -> (wpimath.units.nanoseconds, int, Pose2d):
        if not self.good:
            return None, -1, None
        return (time.monotonic() - self.last_update_time, self.current_target_id, self.robot_in_target_estimated_position)

    def track_disabled(self):
        self.vision_state = _VisionState.VISION_DISABLED

    def track_apriltags(self):
        if self.good:
            self.vision_state = _VisionState.VISION_APRILTAG

    def track_algae(self):
        if self.good:
            self.vision_state = _VisionState.VISION_ALGAE

    def get_algae_data(self) -> (bool, int, Rotation2d):
        if not self.good:
            return None, -1, None
        return self.algae_present, self.algae_last_update_time, self.algae_angle
