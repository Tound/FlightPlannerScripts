#!/usr/bin/env python
"""
Main file to be called to create an efficient flight path for a polygon
and corresponding NFZs for a given terrain
Last updated 30/4/21
"""
import math
import numpy as np
import matplotlib.pyplot as plt
from opensimplex import OpenSimplex

from create_passes import *
from passes_TSP import *
from Image_Classes import *
from camera_calculations import *
from create_spirals import *
import random

import time

class UAV:
    """
    UAV class holds all UAV settings for a specific flight
    """
    def __init__(self,weight,velocity,max_velocity,min_turn,max_incline_grad,heading_angle):
        self.weight = weight                        # Weight of UAV in kg
        self.velocity = velocity                    # Constant velocity of UAV in m/s
        self.max_velocity = max_velocity            # Maximum velocity of UAV in m/s
        self.min_turn = min_turn                    # Minimum turn radius of the UAV in m
        self.max_incline_grad = max_incline_grad    # Maximum incline gradient of the UAV
        self.heading_angle = heading_angle          # Required heading angle of UAV

class Configuration:
    """
    Configuration class holds the settings for an entire flight 
    including Camera object and UAV object
    """
    def __init__(self,uav,camera,side_overlap,ground_sample_distance,wind_angle,altitude,max_pass_length,min_pass_length):
        self.uav = uav                                          # UAV object used in the flight
        self.camera = camera                                    # Camera object used in the flight
        self.side_overlap = side_overlap                        # Side overlap of the images as a decimal
        self.ground_sample_distance = ground_sample_distance    # The desired ground sample distance in m/px
        self.wind = wind                                        # Wind properties in the form (velocity (m/s), direction (radians))
        self.altitude = altitude                                # Desired UAV altitude above the ground
        self.max_pass_length = max_pass_length                  # Maximum pass length in metres
        self.min_pass_length = min_pass_length                  # Minimum pass length in metres



def noise(nx, ny):  # Returns a value from randomly generated noise
    # Rescale from -1.0:+1.0 to 0.0:1.0
    return gen.noise2d(nx, ny) / 2.0 + 0.5

def create_terrain(height,width,freq,multiplier):
    """
    Create randomly generated terrain from user chosen settings
    params:
        height - Height of the area in pixels
        width - Width of the area in pixels
        freq - Frequency of the noise to be used
        multiplier - Value that increases the elevation limits of the generated terrain
    returns:
        terrain - 2D array containing altitude data
    """
    terrain = np.zeros((height,width))  # Initialise 2D array of zeros for size of area
    for y in range(height):             # Cycle through every row
        for x in range(width):          # Cycle through every column
            nx = x/width - 0.5
            ny = y/height - 0.5
            elevation = (multiplier*noise(freq*nx, freq*ny)) # Calculate an value for the elevation
                # Octaves that can be added by uncommenting the following lines
                #+ 2*multiplier*noise(10 * nx, 10 * ny)
                #+ 0*multiplier*noise(20 * nx, 20 * ny)
                #+ 0*5*multiplier*noise(5 * nx, 5 * ny))
            terrain[y][x] =  multiplier*math.pow(elevation,0.5) # Store the calculated elevation value in terrain array

    return terrain

"""
For testing purposes, a set of predefined settings 
for a test case are present here
"""
# Setup the different settings for testing

# UAV settings
min_turn = 10               # The minimum turn radius of the UAV in metres
max_incline_grad = 30       # The maximum incline of the UAV in degrees
glide_slope = 20            # The maximum decline angle of the UAV for a succesful glide slope in degrees
uav_mass = 2.6              # Mass of the UAV in Kg
uav_speed = 15              # The desired constant speed of the UAV in metres per second
uav_max_speed = 20          # The maximum speed of the UAV in metres per second
max_current_draw = 20       # The maximum current draw of the UAV in Amps
battery_capacity = 2200     # The battery capacity in milliAmp Hours

# Camera settings - Altered to match specs similar to the sony a6000
side_overlap = 0.6          # Decimal value of the side overlap of the image footprint
sensor_x = 23.5    *10**-3  # The camera sensor width in mm
sensor_y = 15.6    *10**-3  # The camera sensor height in mm
focal_length = 20  *10**-3  # The focal length of the camera in mm
aspect_ratio = (3,2)        # The aspect ratio of an image taken with the camera in the form x:y
cam_resolution = 24         # The resolution of the camera in Megapixels

# Flight settings
wind = (5,math.radians(90))     # Wind settings as polar coords (Magnitude, direction in radians)
ground_sample_distance = 0.02   # The ground sample distance in metres per pixel
altitude =  None                # The altitude of the UAV above the ground in metres

# Create random terrain using noise
# Arbitrary values for creating random terrain
height = 1000       # 1 km
width = 1200        # 1.2 km
freq = 3            # Hz
multiplier = 15     # Multiplier to increase variation in elevation

gen = OpenSimplex(seed=random.randint(0,100))           # Obtain random noise seed
terrain = create_terrain(height,width,freq,multiplier)  # Generate random terrain

# Set algorithm security factors -  Can be altered depending on the user set limits
# Examples values
max_pass_length = 5000          # Maximum pass length in metres
min_pass_length = 5             # Minimum pass length in metres


# Setup polygon ROI
# Create a polygon vertices to represent the area to be surveyed

polygon = [[100,100],[200,600],[600,900],[1000,900],[800,500],[1100,100]] # Non convex shape


# Define no fly zones vertices
NFZ = [[300,450],[450,450],[450,200],[300,200]]
NFZ2 = [[200,450],[300,450],[300,350],[200,350]]

NFZs = [NFZ,NFZ2]

# Set the start location
start_loc = [400,730,terrain[730][400]]

# Viablility checks to ensure constant ground speed
speed_required = math.sqrt(wind[0]*wind[0] + uav_speed*uav_speed)
if speed_required > uav_max_speed:
    print("Too windy for this flight as the UAV would exceed maximum speed")
    exit(1)
else:
    # Calculate heading angle
    heading_angle = math.atan(wind[0]/uav_speed)
    print(f"Plane will fly with a heading angle of {round(math.degrees(heading_angle),2)} degrees towards the wind!")
    print(f"Required UAV speed: {round(speed_required,2)} m/s")


# Create UAV, camera and configuration object and store all variables
uav = UAV(uav_mass,uav_speed,uav_max_speed,min_turn,max_incline_grad,heading_angle)
camera = Camera(sensor_x,sensor_y,focal_length,cam_resolution,aspect_ratio)
config = Configuration(uav,camera,side_overlap,ground_sample_distance,wind,altitude,max_pass_length,min_pass_length)

# Define all edges for the polygon
polygon_edges = []
for i in range(0,len(polygon)):     # Cycle through each polygon vertex
    # Create a new edge and add to the list of polygon edges
    polygon_edges.append(Edge(polygon[i-1][0],polygon[i-1][1],
                    polygon[i][0],polygon[i][1]))

# Define all edges for the NFZs
NFZ_edges = []
for NFZ in NFZs:                    # Cycle through each NFZ
    for i in range(0,len(NFZ)):     # Cycle through the vertices of the NFZ
        # Create a new edge and add to the list of NFZ edges
        NFZ_edges.append(Edge(NFZ[i-1][0],NFZ[i-1][1],
                        NFZ[i][0],NFZ[i][1]))
            

start_time = time.perf_counter()   # Get current time for measuring the time to create the flight path

# Create passes from the ROI
image_passes = createPasses(polygon,polygon_edges,NFZs,terrain,config) # Create pass objects for current configuration

time_to_create_passes = time.perf_counter()    # Store the time after the create passes algorithm is completed

# Setup the figure for visualising the path
fig = plt.figure(num=1,clear=True,figsize=(12,8))                   # Set size of figure
ax = fig.add_subplot(1,1,1,projection='3d')                         # Set figure to 3D
(x,y) = np.meshgrid(np.arange(0,width,1),np.arange(0,height,1))     # Create mesh grid for the terrain size
ax.plot_surface(x, y, terrain,cmap='terrain',zorder=5)              # Plot the terrain
ax.set(title=f'Flight path created\nWind angle: {round(math.degrees(wind[1]),2)}'\
         ' degs',xlabel='x', ylabel='y', zlabel='z = Height (m)')

ax.set_aspect(aspect='auto')
fig.tight_layout()

# Draw the polygon on the figure
polygon = np.array(polygon)                                     # Convert the array to numpy array for easier use
polygon = np.insert(polygon,0,polygon[len(polygon)-1],axis=0)   # Add first vertex to end to make complete path
plt.plot(polygon[:,0],polygon[:,1],0,'-bo',zorder=15)           # Plot the ROI

# Draw the NFZs on the figure
for NFZ in NFZs:                                                                # Cycle through each NFZ
    NFZ_points = np.array(NFZ)                                                  # Convert the array to numpy array for easier use
    NFZ_points = np.insert(NFZ_points,0,NFZ_points[len(NFZ_points)-1],axis=0)   # Add first vertex to end to make complete path
    plt.plot(NFZ_points[:,0],NFZ_points[:,1],0,'-ro',zorder=15)                 # Plot the NFZ

for image_pass in image_passes:     # Cycle through each image pass
    # Clear the x, y and z points array
    loc_x = np.array([])            
    loc_y = np.array([])
    loc_z = np.array([])
    # Add each coordinate to the correct array
    loc_x = np.append(loc_x,image_pass.start[0])
    loc_x = np.append(loc_x,image_pass.end[0])
    loc_y = np.append(loc_y,image_pass.start[1])
    loc_y = np.append(loc_y,image_pass.end[1])
    loc_z = np.append(loc_z,image_pass.start[2])
    loc_z = np.append(loc_z,image_pass.end[2])
        
    plt.plot(loc_x,loc_y,loc_z,'-ro',zorder=10)     # Plot the points of the pass

# Use TSP to find shortest route
shortest_path = TSP(image_passes,wind[1],min_turn,uav_mass,NFZs,NFZ_edges,max_incline_grad,glide_slope,
                    start_loc,populationSize=30,generations=2000,mutationRate=0.03)

end_time = time.perf_counter() - start_time    # Calculate time taken to create passes and findest shortest route

print("\nFlight path created successfully!\nOutput:\n")
print(shortest_path)    # Print the output to the console

# Print flight stats
print("\nFlight path metrics:")
print(f"Total time to create passes: {round(time_to_create_passes/60,2)} mins")
print(f"Total time to solve the TSP: {round((end_time-time_to_create_passes)/60,2)} mins")
print(f"Total time to complete: {round(end_time/60,2)} mins")
print(f"Total length of route: {round(shortest_path.getLength(),2) }m")
print(f"Total energy of route: {round(shortest_path.getEnergy(),2)}")
print(f"Number of terraces: {len(shortest_path)-1}")


time_of_flight = shortest_path.getLength()/uav_speed
print(f"Estimated time of flight: {round(time_of_flight/60,2)} mins at constant velocity of "
    f"{round(speed_required,2)} m/s resulting in a forward velocity of {uav_speed} m/s")

# Uncomment for simple battery calculations for worst case

#current_used = max_current_draw*time_of_flight/3600
#print(f"Estimated Current draw (Worst case): {round(current_used,2)}A (Current draw of {max_current_draw}A)")
#if current_used > battery_capacity*10**-3:
#    print(f"Current battery capacity of {battery_capacity}mAh will not suffice")
#else:
#    print(f"Current battery capacity {battery_capacity}mAh of will suffice")

spirals =  shortest_path.get_spirals()  # Get all spirals required
dpaths = shortest_path.get_DPaths()     # Get the Dubins paths required for the shortest route

step_size = 1  # Specify step size for sampling each of the created dubins paths

# Sample any dubins paths required in the flight path
for dpath in dpaths:
    # Initialise numpy arrays to store sampled points of the dubins paths
    dubins_x = np.array([])
    dubins_y = np.array([])
    dubins_z = np.array([])  
    points = dubins_path_sample_many(dpath,step_size)    # Sample the dubins paths and obtain the sampled points
    for point in points:
        # Store the coordinate components of the point
        dubins_x = np.append(dubins_x,point[0])
        dubins_y = np.append(dubins_y,point[1])
        dubins_z = np.append(dubins_z,point[2])
    plt.plot(dubins_x,dubins_y,dubins_z,'-yo',zorder=15,markersize = 1)    # Plot the dubins paths

# Sample any spirals required in the flight path
for spiral in spirals:
    # Initialise numpy arrays to store sampled points of the dubins paths
    spiral_x = np.array([])
    spiral_y = np.array([])
    spiral_z = np.array([])  
    points = sample_spiral(spiral,step_size)    # Sample the spiral paths and obtain the sampled points
    for point in points:
        # Store the coordinate components of the point
        spiral_x = np.append(spiral_x,point[0])
        spiral_y = np.append(spiral_y,point[1])
        spiral_z = np.append(spiral_z,point[2])
    plt.plot(spiral_x,spiral_y,spiral_z,'-yo',zorder=15,markersize = 1)    # Plot the spiral paths

plt.show()      # Show the flight path