# Efficient flight path
"""
Main file to be called to create an efficient flight path for a polygon
and corresponding NFZs for a given terrain
"""
import math
import numpy as np
import matplotlib.pyplot as plt
from opensimplex import OpenSimplex

from create_passes_V2 import *
from Passes_TSP_V2 import *
from camera_calculations import *
from Image_Classes_V2 import *

import time

import sys

class Camera:
    """
    Camera class holds all camera settings for a specific flight
    """
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
    """
    UAV class holds all UAV settings for a specific flight
    """
    def __init__(self,weight,velocity,min_turn,max_incline_grad,min_speed = None,max_speed = None):
        self.weight = weight
        self.velocity = velocity
        self.min_turn = min_turn
        self.max_incline_grad = max_incline_grad

class Configuration:
    """
    Configuration class holds the settings for an entire flight 
    including Camera object and UAV object
    """
    def __init__(self,uav,camera,side_overlap,forward_overlap,coverage_resolution,wind_angle):
        self.uav = uav
        self.camera = camera
        self.side_overlap = side_overlap
        self.forward_overlap = forward_overlap
        self.coverage_resolution = coverage_resolution
        self.wind = wind

"""
For testing purposes only, a set of predefined settings 
for a test case are present here
"""
if len(sys.argv) == 2 and sys.argv[1] == 'test':
    ########################
    # CREATE TERRAIN
    ########################
    height = 750        # 750m
    width = 750         # 750m
    freq = 2            # Hz
    multiplier = 10

    gen = OpenSimplex(seed=random.randint(0,100))
    def noise(nx, ny):
        # Rescale from -1.0:+1.0 to 0.0:1.0
        return gen.noise2d(nx, ny) / 2.0 + 0.5

    terrain = np.zeros((height,width))
    for y in range(height):
        for x in range(width):
            nx = x/width - 0.5
            ny = y/height - 0.5
            elevation = (multiplier*noise(freq*nx, freq*ny)
                + 2*multiplier*noise(10 * nx, 10 * ny)
                + 0*multiplier*noise(20 * nx, 20 * ny)
                + 0*5*multiplier*noise(5 * nx, 5 * ny))
            terrain[y][x] =  multiplier*math.pow(elevation,0.5)

    ######################
    # Setup
    ######################
    # UAV settings
    min_turn = 10 #m
    max_incline_grad = 31 #degs
    glide_slope = 20
    uav_mass = 18 # Kg
    uav_speed = 8

    # Camera settings
    side_overlap = 0.2          # Percentage
    forward_overlap = 0.50       # Percentage
    sensor_x = 5.62    *10**-3  # mm
    sensor_y = 7.4     *10**-3  # mm
    focal_length = 3.6 *10**-3  # mm
    aspect_ratio = (4,3)        # x:y
    cam_resolution = 12         # MP
    image_x = 4000              # px
    image_y = 3000              # px
    fov = 20                    # degs

    # Flight settings
    wind = (5,math.radians(45)) #Polar coords (Mag, degrees)
    coverage_resolution = 0.02  # m/px

    max_current_draw = 20
    battery_capacity = 2200

    # Test cases
    polygon = [[100,100],[100,650],[650,650],[750,350],[650,100]]
    NFZ = [[300,450],[450,450],[450,200],[300,200]]
    NFZ2 = [[200,450],[300,450],[300,350],[200,350]]
    NFZs = []#[NFZ,NFZ2]
    start_loc = [400,730,terrain[730][400]]

else:
    # Convert bearing to angle by doing 90-bearing
    print("Reading text file")


# Viablility checks
if wind[0] > uav_speed:
    print("Too windy for this flight")
    exit(1)
elif wind[0] > uav_speed/2:
    print("Heading angle will be steep")

# Calculate heading angle
heading_angle = math.asin(wind[0]/uav_speed)
print(f"Plane will fly with a heading angle of {round(math.degrees(heading_angle),2)} degrees towards the wind!")

# Create UAV, camera and configuration object and store all variables
uav = UAV(uav_mass,uav_speed,min_turn,max_incline_grad)
camera = Camera(sensor_x,sensor_y,focal_length,cam_resolution,aspect_ratio,image_x,image_y)
config = Configuration(uav,camera,side_overlap,forward_overlap,coverage_resolution,wind)

polygon_edges = []
for i in range(0,len(polygon)):
    polygon_edges.append(Edge(polygon[i-1][0],polygon[i-1][1],
                    polygon[i][0],polygon[i][1]))

NFZ_edges = []
for NFZ in NFZs:
    for i in range(0,len(NFZ)):
        NFZ_edges.append(Edge(NFZ[i-1][0],NFZ[i-1][1],
                        NFZ[i][0],NFZ[i][1]))
            
# Create canvas/ choose area
# Get startpoint, 2D points from canvas and elevation data via intermediate text file + flight settings
# Read file and create points

# Polygon
# NFZ
# UAV Settings
# Camera settings
# Flight settings

# Multiple angles?

start_time = time.clock()   # Get current time for measuring solve time

# Create passes from the ROI
image_passes = createPasses(polygon,polygon_edges,NFZs,terrain,config) # Create pass objects for current configuration

# DRAW FOR TESTING
fig = plt.figure(num=1,clear=True,figsize=(12,8))
ax = fig.add_subplot(1,1,1,projection='3d')
(x,y) = np.meshgrid(np.arange(0,width,1),np.arange(0,height,1))
ax.plot_surface(x, y, terrain,cmap='terrain',zorder=5)
ax.set(title=f'Terrain Generated\nWind angle: {round(math.degrees(wind[1]),2)} degs',xlabel='x', ylabel='y', zlabel='z = Height (m)')
#ax.set_zlim(0,150)

ax.set_aspect(aspect='auto')
fig.tight_layout()

polygon = np.array([[100,100],[100,650],[650,650],[750,350],[650,100]])
NFZ = np.array([[300,450],[450,450],[450,200],[300,200]])
plt.plot(polygon[:,0],polygon[:,1],100,'-bo',zorder=15)
plt.plot(NFZ[:,0],NFZ[:,1],'-ro')

for image_pass in image_passes:
    loc_x = np.array([])
    loc_y = np.array([])
    loc_z = np.array([])
    loc_x = np.append(loc_x,image_pass.start[0])
    loc_x = np.append(loc_x,image_pass.end[0])
    loc_y = np.append(loc_y,image_pass.start[1])
    loc_y = np.append(loc_y,image_pass.end[1])
    loc_z = np.append(loc_z,image_pass.start[2])
    loc_z = np.append(loc_z,image_pass.end[2])
        
    plt.plot(loc_x,loc_y,loc_z,'-ro',zorder=10)

# Need GPS and altitude before TSP
# Update passes with altitudes from API

# Use TSP to find shortest route
shortest_path = TSP(image_passes,wind[1],min_turn,uav_mass,NFZs,NFZ_edges,max_incline_grad,glide_slope,
                    start_loc,populationSize=50,generations=200,mutationRate=0.5)

end_time = time.clock() - start_time    # Calculate time taken to create passes and findest shortest route

# Print flight stats
print(f"Total time to solve: {round(end_time/60,2)}mins")
print(f"Total length of route: {round(shortest_path.getLength(),2)}m")
print(f"Total energy of route: {round(shortest_path.getEnergy(),2)}")

time_of_flight = shortest_path.getLength()/uav_speed
print(f"Estimated time of flight: {round(time_of_flight/60,2)}mins")
current_used = max_current_draw*time_of_flight/3600
print(f"Estimated Current draw (Worst case): {round(current_used,2)}A")
if current_used > battery_capacity*10**-3:
    print("Current battery capacity will not suffice")
else:
    print("Current battery capacity will suffice")

dpaths = shortest_path.getDPaths()  # Get the Dubins paths that make up the shortest route

stepSize = 0.5  # Specify step size for sampling each dubins path

for dpath in dpaths:
    dubinsX = np.array([])
    dubinsY = np.array([])
    dubinsZ = np.array([])  
    points = dubins_path_sample_many(dpath,stepSize)
    for point in points:
        dubinsX = np.append(dubinsX,point[0])
        dubinsY = np.append(dubinsY,point[1])
        dubinsZ = np.append(dubinsZ,point[2])
    plt.plot(dubinsX,dubinsY,dubinsZ,'-yo',zorder=15,markersize = 1)

plt.show()

# Convert into GPS coords
# Requires API for elevation
# Create waypoints file
