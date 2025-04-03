import pygame
import math
import numpy as np
from pygame import gfxdraw
from networktables import NetworkTables
import threading
import time
import wpilib
from wpimath.geometry import Rotation2d, Pose2d, Translation2d


# Field dimensions in meters (adjust according to 2025 field)
FIELD_LENGTH = 16.54175  # meters
FIELD_WIDTH = 8.0137     # meters

# Initialize pygame
pygame.init()
WIDTH, HEIGHT = 1200, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("FRC Robot Position Display")

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
GRAY = (100, 100, 100)

# Scaling factors
SCALE_X = WIDTH / FIELD_LENGTH
SCALE_Y = HEIGHT / FIELD_WIDTH

# Robot pose storage
robot_pose = [0, 0, 0]  # x, y, rotation (radians)
pose_lock = threading.Lock()

# NetworkTables callback
def value_changed(table, key, value, isNew):
    global robot_pose
    if key == "RobotPose":
        with pose_lock:
            robot_pose = value

# Connect to NetworkTables
def nt_init():
    NetworkTables.initialize(server="10.87.88.1")  
    sd = NetworkTables.getTable("SmartDashboard")
    sd.addEntryListener(value_changed)

# Draw field elements
def draw_field():
    # Field border
    pygame.draw.rect(screen, WHITE, (0, 0, WIDTH, HEIGHT), 2)
    
    # Example: Draw center line
    pygame.draw.line(screen, WHITE, (WIDTH//2, 0), (WIDTH//2, HEIGHT), 2)
    
    # Draw AprilTag positions (simplified)
    for i, tag in enumerate(TAG_LIST):
        x = tag.X() * SCALE_X
        y = tag.Y() * SCALE_Y
        pygame.draw.circle(screen, BLUE, (int(x), int(y)), 10)
        font = pygame.font.SysFont(None, 24)
        text = font.render(str(i+1), True, WHITE)
        screen.blit(text, (int(x)+15, int(y)-10))

# Draw robot
def draw_robot():
    with pose_lock:
        x, y, rot = robot_pose
    
    # Convert from field coordinates to screen coordinates
    screen_x = x * SCALE_X
    screen_y = y * SCALE_Y
    
    # Draw robot position
    pygame.draw.circle(screen, RED, (int(screen_x), int(screen_y)), 15)
    
    # Draw orientation line
    end_x = screen_x + 30 * math.cos(rot)
    end_y = screen_y + 30 * math.sin(rot)
    pygame.draw.line(screen, GREEN, (screen_x, screen_y), (end_x, end_y), 3)

# Main loop
def main():
    # Start NetworkTables thread
    nt_thread = threading.Thread(target=nt_init)
    nt_thread.daemon = True
    nt_thread.start()
    
    clock = pygame.time.Clock()
    running = True
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Clear screen
        screen.fill(BLACK)
        
        # Draw field
        draw_field()
        
        # Draw robot
        draw_robot()
        
        # Display coordinates
        with pose_lock:
            font = pygame.font.SysFont(None, 36)
            text = font.render(f"X: {robot_pose[0]:.2f}m Y: {robot_pose[1]:.2f}m Rot: {math.degrees(robot_pose[2]):.1f}°", True, WHITE)
            screen.blit(text, (20, 20))
        
        pygame.display.flip()
        clock.tick(30)
    
    pygame.quit()

if __name__ == "__main__":
    TAG_LIST = [
        # Only including key tags for testing
        # Format: Pose2d(X_meters, Y_meters, Rotation_radians)
        
        # Red Alliance Tags (simplified)
        Pose2d(15.98, 0.66, Rotation2d.fromDegrees(120)),   # Tag ID 1
        Pose2d(15.98, 4.42, Rotation2d.fromDegrees(60)),    # Tag ID 2
        
        # Blue Alliance Tags (simplified)
        Pose2d(0.56, 0.66, Rotation2d.fromDegrees(300)),    # Tag ID 4
        Pose2d(0.56, 4.42, Rotation2d.fromDegrees(240)),    # Tag ID 5
        
        # Center Tags (simplified)
        Pose2d(8.27, 4.10, Rotation2d.fromDegrees(180)),    # Tag ID 3
        Pose2d(8.27, 0.90, Rotation2d.fromDegrees(0)),      # Tag ID 6
    ]
    main()