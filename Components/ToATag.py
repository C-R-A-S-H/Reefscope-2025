import logging
from typing import List, Tuple, Optional
from wpilib import DriverStation
# from pathplannerlib import PathPlannerPath, PathPoint
from wpimath.geometry import Translation2d
from networktables import NetworkTables

class PathPlannerPath:
    @staticmethod
    def from_path_points(points, constraints, goal_end_state):
        return PathPlannerPath()

    def get_all_path_points(self):
        return []


# Placeholder for PathPoint
class PathPoint:
    def __init__(self, position: Translation2d, heading: Optional[float] = None):
        self.position = position
        self.heading = heading

class LocalADStarAK:
    """
    High-level control class for the AD* pathfinding algorithm.
    Handles path retrieval, updates, and state management.
    """

    def __init__(self):
        # Initialize AD* IO handler
        self.io = ADStarIO()

    def is_new_path_available(self) -> bool:
        """
        Check if a new path is available. Update state only if not in Replay mode.
        """
        if not DriverStation.isReplay():
            self.io.update_is_new_path_available()
        return self.io.is_new_path_available

    def get_current_path(self, constraints, goal_end_state: 'GoalEndState') -> Optional[PathPlannerPath]:
        """
        Retrieve the current path as a PathPlannerPath object.
        Returns None if no path points are available.
        """
        if not DriverStation.isReplay():
            self.io.update_current_path_points(constraints, goal_end_state)

        if not self.io.current_path_points:
            logging.warning("No path points available. Returning None.")
            return None

        return PathPlannerPath.from_path_points(self.io.current_path_points, constraints, goal_end_state)

    def set_start_position(self, start_position: Translation2d):
        """
        Set the starting position for the AD* pathfinding algorithm.
        """
        if not DriverStation.isReplay():
            self.io.ad_star.set_start_position(start_position)

    def set_goal_position(self, goal_position: Translation2d):
        """
        Set the goal (target) position for the AD* algorithm.
        """
        if not DriverStation.isReplay():
            self.io.ad_star.set_goal_position(goal_position)

    def set_dynamic_obstacles(self, obs: List[Tuple[Translation2d, Translation2d]], current_robot_pos: Translation2d):
        """
        Set dynamic obstacles for the AD* algorithm, using a list of obstacle pairs.
        Ensures function is skipped in Replay mode.
        """
        if not DriverStation.isReplay():
            self.io.ad_star.set_dynamic_obstacles(obs, current_robot_pos)


class ADStarIO:
    """
    Data handling and I/O for the LocalADStar algorithm.
    Handles data interfacing with NetworkTables and internal state updates.
    """

    def __init__(self):
        # Initialize the LocalADStar instance
        self.ad_star = LocalADStar()
        # State variables
        self.is_new_path_available = False
        self.current_path_points: List[PathPoint] = []

    def to_log(self):
        """
        Log internal state to NetworkTables for external monitoring.
        """
        table = NetworkTables.getTable("LocalADStarAK")

        # Log whether a new path is available
        table.putBoolean("IsNewPathAvailable", self.is_new_path_available)

        # Convert path points into flat x, y coordinate array
        points_logged = []
        for point in self.current_path_points:
            points_logged.extend([point.position.x, point.position.y])

        table.putNumberArray("CurrentPathPoints", points_logged)

    def from_log(self):
        """
        Retrieve internal state from NetworkTables.
        Retrieves both path availability and current path points.
        """
        table = NetworkTables.getTable("LocalADStarAK")

        # Retrieve whether a new path is available
        self.is_new_path_available = table.getBoolean("IsNewPathAvailable", False)

        # Retrieve the path points and rebuild the PathPoint list
        points_logged = table.getNumberArray("CurrentPathPoints", [])
        self.current_path_points = [
            PathPoint(Translation2d(points_logged[i], points_logged[i + 1]), None)
            for i in range(0, len(points_logged), 2)
        ]

    def update_is_new_path_available(self):
        """
        Update the internal state for "is new path available".
        """
        self.is_new_path_available = self.ad_star.is_new_path_available()

    def update_current_path_points(self, constraints, goal_end_state: 'GoalEndState'):
        """
        Fetch and update the current path points from the AD* algorithm.
        Clears the points if no valid path is found.
        """
        current_path = self.ad_star.get_current_path(constraints, goal_end_state)

        if current_path:
            self.current_path_points = current_path.get_all_path_points()
        else:
            self.current_path_points = []


class LocalADStar:
    """
    Core pathfinding algorithm placeholder.
    Replace with your actual AD* algorithm implementation or logic.
    """

    def set_start_position(self, start_position: Translation2d):
        # Set the start position for pathfinding
        pass

    def set_goal_position(self, goal_position: Translation2d):
        # Set the goal position for pathfinding
        pass

    def set_dynamic_obstacles(self, obs: List[Tuple[Translation2d, Translation2d]], current_robot_pos: Translation2d):
        # Set dynamic obstacles for pathfinding
        pass

    def get_current_path(self, constraints, goal_end_state: 'GoalEndState') -> Optional[PathPlannerPath]:
        # Return the current path (or None if no path exists)
        return None

    def is_new_path_available(self) -> bool:
        # Return whether a new path is available
        return True
