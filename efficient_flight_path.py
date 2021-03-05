# Efficient flight path
"""
Main file to be called to create an efficient flight path for a polygon
"""
import math
import numpy as np
import matplotlib.pyplot as plt
from opensimplex import OpenSimplex

from create_passes import *
from Modified_Dubins_TSP import *
from camera_calculations import *

class Imaging_Pass:
    def __init__(self,start,end):
        self.start = start
        self.end = end
        self.image_locs = []
        self.energy = 0

    def getEnergy(self):
        return self.energy

class Image_Location:
    def __init__(self,x,y,altitude):
        self.x = x
        self.y = y
        self.altitude = altitude

class Camera:
    def __init__(self,sensor_x,sensor_y,focal_length,resolution,aspect_ratio,image_x=None,image_y=None,fov=None):
        self.sensor_x = sensor_x
        self.sensor_y = sensor_y
        self.focal_length = focal_length
        self.resolution = resolution
        self.aspect_ratio = aspect_ratio
        self.image_x = image_x
        self.image_y = image_y
        self.fov = fov

class UAV:
    def __init__(self,weight,velocity,min_turn,max_incline_grad,min_speed = None,max_speed = None):
        self.weight = weight
        self.velocity = velocity
        self.min_turn = min_turn
        self.max_incline_grad = max_incline_grad

class Configuration:
    def __init__(self,uav,camera,side_overlap,forward_overlap,coverage_resolution,wind_angle):
        self.uav = uav
        self.camera = camera
        self.side_overlap = side_overlap
        self.forward_overlap = forward_overlap
        self.coverage_resolution = coverage_resolution
        self.wind = wind

########################
# CREATE TERRAIN
########################
height = 750        # 750m
width = 750         # 750m
freq = 2            # Hz

gen = OpenSimplex(seed=random.randint(0,100))
def noise(nx, ny):
    # Rescale from -1.0:+1.0 to 0.0:1.0
    return gen.noise2d(nx, ny) / 2.0 + 0.5

terrain = np.zeros((width,height))
for y in range(height):
    for x in range(width):
        nx = x/width - 0.5
        ny = y/height - 0.5
        elevation = 20*noise(freq*nx, freq*ny)
        terrain[y][x] =  20*math.pow(elevation,0.5)

######################
# Setup
######################
# UAV settings
min_turn = 25 #m
max_incline_grad = 30 #degs
uav_mass = 18 # Kg
uav_speed = 50

# Camera settings
side_overlap = 0.8          # Percentage
forward_overlap = 0.6       # Percentage
sensor_x = 5.62    *10**-3  # mm
sensor_y = 7.4     *10**-3  # mm
focal_length = 3.6 *10**-3  # mm
aspect_ratio = (4,3)        # x:y
cam_resolution = 12         # MP
image_x = 4000              # px
image_y = 3000              # px
fov = 20                    # degs

# Flight settings
wind = (10,math.radians(-45)) #Polar coords (Mag, degrees)
coverage_resolution = 0.05  # m/px

uav = UAV(uav_mass,uav_speed,min_turn,max_incline_grad)
camera = Camera(sensor_x,sensor_y,focal_length,cam_resolution,aspect_ratio,image_x,image_y)
config = Configuration(uav,camera,side_overlap,forward_overlap,coverage_resolution,wind)

# Test cases
polygon = [[100,100],[100,650],[650,650],[650,100]]
NFZ = [[200,450],[450,450],[450,200],[200,200]]
NFZs = []
start_loc = [20,730,terrain[20][730]]

#altitude = getAltitude(focal_length,coverage_x,sensor_x)
# Create canvas/ choose area
# Get startpoint, 2D points from canvas and elevation data via intermediate text file + flight settings
# Read file and create points

# Polygon
# NFZ
# UAV Settings
# Camera settings
# Flight settings

# Split cells into passes
# Multiple angles?
image_passes = createPasses(polygon,NFZs,terrain,config)

# Use tsp to find shortest route
shortest_path = GTSP()
for point in shortest_path:
    print("point")

# Convert into GPS coords

# Create waypoints file