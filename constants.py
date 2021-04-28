# Simulation Parameters
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
PIX2M = 0.01  # factor to convert from pixels to meters
M2PIX = 100.0  # factor to convert from meters to pixels
NUM_DRONES = 1 # Number of simultaneous drones
SIZE_DRONE = 18
SIZE_TRACK = 600
RESOLUTION = 100 # Of grid
# Sample Time Parameters
FREQUENCY = 60.0  # simulation frequency
SAMPLE_TIME = 1.0 / FREQUENCY  # simulation sample time

# Behavior Parameters
FORWARD_SPEED = 3  # default linear speed when going forward
ANGULAR_SPEED = 1.5# default angular speed
SEEK_FORCE = 0.1 # max seek force
RADIUS_TARGET = 130 
MASS = 1 # Drone Mass, used to calculate force
HOP_AHEAD = 60 # distance of prevision
AVOID_DISTANCE = 40 # distance to avoid collision

# Colors
BLACK = (0,0,0)
LIGHT_BLUE = (224, 255, 255)
BLUE = (0,0,255)
RED = (255,0,0)