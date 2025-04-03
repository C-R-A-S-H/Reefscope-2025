import pygame
import math
import numpy as np
from pygame import gfxdraw
from networktables import NetworkTables
import threading
import time
from collections import deque
from wpimath.geometry import Pose2d, Rotation2d
import colorsys

# Field dimensions in meters (adjust according to 2025 field)
FIELD_LENGTH = 16.54175  # meters
FIELD_WIDTH = 8.0137     # meters

# Initialize pygame with anti-aliasing
pygame.init()
WIDTH, HEIGHT = 1200, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.SCALED)
pygame.display.set_caption("FRC Robot Vision Display - Team XXXX")
clock = pygame.time.Clock()

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 50, 50)
BLUE = (50, 50, 255)
GREEN = (50, 255, 50)
GRAY = (100, 100, 100)

# Scaling factors
SCALE_X = WIDTH / FIELD_LENGTH
SCALE_Y = HEIGHT / FIELD_WIDTH

# Robot pose storage
robot_pose = [0, 0, 0]  # x, y, rotation (radians)
current_tag_id = -1
last_tag_update = 0
pose_lock = threading.Lock()

# Animation variables
tag_pulse_phase = 0
fps_history = deque(maxlen=60)
smooth_fps = 60

# Font setup
pygame.font.init()
font_large = pygame.font.Font(None, 36)
font_medium = pygame.font.Font(None, 28)
font_small = pygame.font.Font(None, 24)

# NetworkTables callback
def value_changed(table, key, value, isNew):
    global robot_pose, current_tag_id, last_tag_update
    if key == "RobotPose":
        with pose_lock:
            robot_pose = value
    elif key == "CurrentTagId":
        with pose_lock:
            current_tag_id = value
            last_tag_update = time.time()

# Connect to NetworkTables
def nt_init():
    NetworkTables.initialize(server="roboRIO-XXXX-FRC.local")  # Replace with your team number
    sd = NetworkTables.getTable("SmartDashboard")
    sd.addEntryListener(value_changed)
    # Request tag ID updates from robot
    sd.putNumber("RequestTagUpdates", 1)

# Smoothing function for animations
def smoothstep(t):
    return t * t * (3 - 2 * t)

# Get rainbow color based on phase (0-1)
def get_rainbow_color(phase):
    hue = (phase % 1.0)  # Cycle through hue spectrum
    r, g, b = colorsys.hsv_to_rgb(hue, 0.8, 1.0)
    return (int(r*255), int(g*255), int(b*255))

# Draw smooth circle with anti-aliasing
def draw_smooth_circle(surface, color, pos, radius, width=1):
    gfxdraw.aacircle(surface, int(pos[0]), int(pos[1]), radius, color)
    if width > 1:
        gfxdraw.filled_circle(surface, int(pos[0]), int(pos[1]), radius-width, color)

# Draw field elements with smooth animations
def draw_field():
    # Field border with subtle glow
    for i in range(3, 0, -1):
        glow_color = (100, 100, 100, 50//i)
        pygame.draw.rect(screen, glow_color, (-i, -i, WIDTH+i*2, HEIGHT+i*2), 2)
    pygame.draw.rect(screen, WHITE, (0, 0, WIDTH, HEIGHT), 2)
    
    # Center line with gradient
    for x in range(WIDTH//2 - 2, WIDTH//2 + 3):
        alpha = 255 - abs(x - WIDTH//2) * 50
        line_color = (255, 255, 255, alpha)
        pygame.draw.line(screen, line_color, (x, 0), (x, HEIGHT), 1)

# Draw AprilTags with pulsing rainbow effect
def draw_apriltags():
    global tag_pulse_phase
    
    # Update animation phase
    tag_pulse_phase += 0.01
    if tag_pulse_phase > 1.0:
        tag_pulse_phase = 0.0
    
    for i, tag in enumerate(TAG_LIST):
        # Convert from field coordinates to screen coordinates
        screen_x = tag.X() * SCALE_X
        screen_y = tag.Y() * SCALE_Y
        
        # Pulsing effect for active tag
        is_active = (i+1 == current_tag_id)
        pulse_size = 5 if is_active else 0
        pulse_alpha = 100 if is_active else 30
        
        # Rainbow glow for active tag
        if is_active:
            for r in range(30, 15, -5):
                rainbow_color = get_rainbow_color(tag_pulse_phase + r/100)
                draw_smooth_circle(screen, (*rainbow_color, pulse_alpha), 
                                 (screen_x, screen_y), 25 + r + pulse_size)
        
        # Tag background
        tag_color = BLUE if not is_active else GREEN
        draw_smooth_circle(screen, (*tag_color, 200), (screen_x, screen_y), 25)
        
        # Tag ID text
        tag_text = font_medium.render(str(i+1), True, WHITE)
        text_rect = tag_text.get_rect(center=(screen_x, screen_y))
        screen.blit(tag_text, text_rect)
        
        # Last seen indicator
        if is_active:
            time_since_seen = time.time() - last_tag_update
            if time_since_seen < 2:  # Only show if recently seen
                alpha = max(0, 255 - int(time_since_seen * 128))
                pygame.draw.circle(screen, (*GREEN, alpha), 
                                 (int(screen_x), int(screen_y)), 
                                 30 + int(10 * math.sin(time.time()*5)), 2)

# Draw robot with smooth orientation
def draw_robot():
    with pose_lock:
        x, y, rot = robot_pose
    
    # Convert from field coordinates to screen coordinates
    screen_x = x * SCALE_X
    screen_y = y * SCALE_Y
    
    # Robot body with shadow
    pygame.draw.circle(screen, (50, 50, 50, 100), (int(screen_x)+3, int(screen_y)+3), 18)
    draw_smooth_circle(screen, RED, (screen_x, screen_y), 15)
    
    # Orientation indicator with smooth movement
    end_x = screen_x + 30 * math.cos(rot)
    end_y = screen_y + 30 * math.sin(rot)
    pygame.draw.line(screen, GREEN, (screen_x, screen_y), (end_x, end_y), 3)
    
    # Robot status text
    robot_text = font_small.render(f"Robot", True, WHITE)
    text_rect = robot_text.get_rect(center=(screen_x, screen_y-25))
    screen.blit(robot_text, text_rect)

# Draw HUD with performance metrics
def draw_hud():
    # Calculate smoothed FPS
    fps = clock.get_fps()
    fps_history.append(fps if fps > 0 else 60)
    global smooth_fps
    smooth_fps = sum(fps_history) / len(fps_history)
    
    # Background panel
    pygame.draw.rect(screen, (*BLACK, 150), (10, 10, 300, 110))
    
    # Title
    title_text = font_large.render("Vision Display", True, WHITE)
    screen.blit(title_text, (20, 15))
    
    # Data display
    with pose_lock:
        x, y, rot = robot_pose
        data_lines = [
            f"Position: {x:.2f}m, {y:.2f}m",
            f"Heading: {math.degrees(rot):.1f}°",
            f"Tracking: {'Tag #'+str(current_tag_id) if current_tag_id != -1 else 'None'}",
            f"FPS: {smooth_fps:.1f}"
        ]
    
    for i, line in enumerate(data_lines):
        text = font_medium.render(line, True, WHITE)
        screen.blit(text, (20, 50 + i*20))

# Main loop
def main():
    # Start NetworkTables thread
    nt_thread = threading.Thread(target=nt_init)
    nt_thread.daemon = True
    nt_thread.start()
    
    running = True
    
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Clear screen with subtle gradient
        for y in range(HEIGHT):
            shade = 10 + int(5 * (y/HEIGHT))
            pygame.draw.line(screen, (shade, shade, shade), (0, y), (WIDTH, y))
        
        # Draw all elements
        draw_field()
        draw_apriltags()
        draw_robot()
        draw_hud()
        
        # Update display
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()

if __name__ == "__main__":
    # Define your TAG_LIST here (use the simplified or full version)
    TAG_LIST = [
        # Your tag definitions here
        Pose2d(15.98, 0.66, Rotation2d.fromDegrees(120)),   # Tag ID 1
        Pose2d(15.98, 4.42, Rotation2d.fromDegrees(60)),    # Tag ID 2
        # Add more tags as needed
    ]
    main()