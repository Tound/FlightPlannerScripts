#3D Coverage
import matplotlib.pyplot as plt
import matplotlib.path as path
import matplotlib.patches as patches
import math
import numpy as np
import random
from opensimplex import OpenSimplex
from Modified_Dubins_TSP import *


# CREATE TERRAIN

height = 750
width = 750
freq = 2
octaves = (0.1,0.5,1)

gen = OpenSimplex(seed=random.randint(0,100))
def noise(nx, ny):
    # Rescale from -1.0:+1.0 to 0.0:1.0
    return gen.noise2d(nx, ny) / 2.0 + 0.5

value = np.zeros((width,height))
for y in range(height):
    for x in range(width):
        nx = x/width - 0.5
        ny = y/height - 0.5
        elevation = 20*noise(freq*nx, freq*ny)
        #+ 1/octaves[0]*noise(1/octaves[0] * nx, 1/octaves[0] * ny) 
        #+ 1/octaves[1]*noise(1/octaves[1] * nx, 1/octaves[1] * ny) 
        #+ 1/octaves[2]*noise(1/octaves[2] * nx, 1/octaves[2] * ny)
        value[y][x] =  20*math.pow(elevation,0.5)

fig = plt.figure(num=1,clear=True,figsize=(12,8))
ax = fig.add_subplot(1,1,1,projection='3d')

(x,y) = np.meshgrid(np.arange(0,width,1),np.arange(0,height,1))
ax.plot_surface(x, y, value,cmap='terrain')
ax.set(title='Terrain Generated',xlabel='x', ylabel='y', zlabel='z = Height (m)')
#ax.set_zlim(0,150)

#z = 0.5(cos(x-0.4))^2 - 0.2x + 0.1sin(y^2) -0.01x^2 -0.1y
#z = 0.000000000005*(0.00008*np.cos(0.00000006*x-0.004))**0.00002 - 0.00000002*x + 0.0000001*np.sin(0.000008*y**0.000002) -0.00001*x**2 -0.00001*y

# Flight setup

imaging_passes  = []
all_image_locations = []


class imaging_pass:
    def __init__(self, start, end, img_locs=None):
        self.img_locs = img_locs
        self.start = start
        self.end = end

        dx = end[0] - start[0]
        dy = end[1] - start[1]
        self.length = math.sqrt(dx*dx +dy*dy)

    def add_img_locs(self, img_locs):
        self.img_locs = img_locs

class image_loc:
    def __init__(self,x,y,altitude):
        self.x = x
        self.y = y
        self.altitude = altitude

# UAV settings
min_turn = 2 #m
max_incline_grad = 30 #degs
# Wind settings
wind = (10,math.radians(-45)) #Polar coords (Mag, degrees)

# Camera settings
side_overlap = 0.5          # Percentage
forward_overlap = 0.4       # Percentage
sensor_x = 5.62    *10**-3  # mm
sensor_y = 7.4     *10**-3  # mm
focal_length = 3.6 *10**-3  # mm
aspect_ratio = (4,3)        # x:y
cam_resolution = 12         # MP
image_x = 4000              # px
image_y = 3000              # px

# Flight settings`
coverage_resolution = 0.05  # m/px

# Start/end location
start_pos = (0,0)

# AOI
polygon = [[100,100],[100,650],[650,650],[650,100]]
polygon.append(polygon[0])

start_loc = [20,20,value[20][20]]


NFZ = [[200,450],[450,450],[450,200],[200,200]]
NFZ.append(NFZ[0])

coverage_x = coverage_resolution * image_x
coverage_y = coverage_resolution * image_y

uav_altitude = coverage_x *focal_length/sensor_x      # Above ground

distance_between_photos_x = coverage_x - coverage_x*side_overlap
distance_between_photos_y = coverage_y - coverage_y*forward_overlap

# Create image positions
wind_dx = math.cos(wind[1])
wind_dy = math.sin(wind[1])
wind_grad = wind_dy/wind_dx
pass_grad = -1/wind_grad

pass_angle = math.atan(pass_grad)

# Find max and min poly point relative to wind
# Max and min relative the value y intercept
# Max and min x used to find the width of the polygon
min_intercept = math.inf
max_intercept = -math.inf
max_x = -math.inf
min_x = math.inf

for vertex in polygon:
    intercept = vertex[1] - pass_grad*vertex[0] # C = y-mx
    if intercept > max_intercept:
        max_intercept = intercept
        max_point = (vertex[0],vertex[1])
    elif intercept < min_intercept:
        min_intercept = intercept
        min_point = (vertex[0],vertex[1])
    if vertex[0] > max_x:
        max_x = vertex[0]
    elif vertex[0] < min_x:
        min_x = vertex[0]

# Find distance between max line and min line
# Create intercepting line for both, start at max line
# Use wind gradient to compare (x,max_point) and (x, min_point)
"""
y = m1x+C1
y = m2x+C2
(m1-m2)x = C2-C1
x = (C2-C1)/(m1-m2)  
"""
x = (min_intercept-max_intercept)/(wind_grad-pass_grad)
y = x*pass_grad + min_intercept

dx = x
dy = max_intercept-y
length = math.sqrt(dx*dx + dy*dy)

#number_of_passes = int((length-coverage_x)/coverage_x)
number_of_passes = int((length-0*coverage_x/2)/distance_between_photos_x) +1 # Passes above the first pass

pass_shift = (coverage_x/2 + (length-(coverage_x/2 + number_of_passes*distance_between_photos_x)))/2

print(pass_shift,coverage_x,distance_between_photos_x, length,number_of_passes)

x_range = np.arange(round(min_x),round(max_x),1)

conv_factor = (max_intercept-min_intercept)/length  # Maps changes within the lines to the length of the y axis

polygon_path = path.Path(polygon)
nfz_path = path.Path(NFZ)
pass_points = []

y_shift = min_intercept + pass_shift*conv_factor # First pass is on the shift

# Find the points that are in the polygon
for i in range(number_of_passes-1):
    y_shift += conv_factor*distance_between_photos_x
    x_points = np.array([])
    points = []
    for j in range(0,len(x_range)):
        y = pass_grad*x_range[j] + y_shift
        if polygon_path.contains_point((x_range[j],y)): # Be smarter!! Only need start and end
            x_points = np.append(x_points, x_range[j])
            points.append((x_range[j],y))
    imaging_passes.append(imaging_pass(points[0],points[len(points)-1]))
    pass_points.append(points)


# Draw the polygon

poly_x = np.array([])
poly_y = np.array([])
for vertex in polygon:
    poly_x = np.append(poly_x,vertex[0])
    poly_y = np.append(poly_y,vertex[1])

plt.plot(poly_x,poly_y,'-bo')

nfz_x = np.array([])
nfz_y = np.array([])
for vertex in NFZ:
    nfz_x = np.append(nfz_x,vertex[0])
    nfz_y = np.append(nfz_y,vertex[1])

plt.plot(nfz_x,nfz_y,'-bo')

plt.plot(start_loc[0],start_loc[1],start_loc[2],'ro',markersize=3)

x_shift = distance_between_photos_y*math.cos(pass_angle)
y_shift = distance_between_photos_y*math.sin(pass_angle)
# Find image locations 
for imaging_pass in imaging_passes: # Cycle through all passes
    length = imaging_pass.length
    number_of_image_locs = int((length-0*coverage_y/2) / distance_between_photos_y) +1 # Number of image locations on pass
    start_shift = (coverage_y/2 + (length-(coverage_y/2 + (number_of_image_locs)*distance_between_photos_y)))/2

    image_locations = []
    x = imaging_pass.start[0] + start_shift*math.cos(pass_angle)
    y = imaging_pass.start[1] + start_shift*math.sin(pass_angle)
    for i in range(number_of_image_locs-1):
        x += x_shift
        y += y_shift

        if nfz_path != None  and nfz_path.contains_point((x,y)):
            continue
        altitude = uav_altitude + value[int(y)][int(x)]
        #print(altitude)
        image_loc(x,y,altitude) # Create new image location
        image_locations.append(image_loc) # Add object to list
        # Add to long list of all image locations
        all_image_locations.append((x,y,altitude))   # 2D in this case
        plt.plot(x,y,altitude,'-bo',markersize=3)
    imaging_pass.add_img_locs(image_locations)

# TSP 

all_image_locations.insert(0,start_loc)

routemanager = RouteManager()
for location in all_image_locations:
    image_location = ImageLocation(location[0],location[1],location[2])
    routemanager.addImageLocation(image_location)

# Initialize population
pop = Population(routemanager, routemanager.numberOfLocations(), True)
print( "Initial distance: " + str(pop.getFittest().getEnergy()))

# Evolve population for 50 generations
ga = GA(routemanager,0.015,20)
pop = ga.evolvePopulation(pop)
for i in range(0, 10000):
    pop = ga.evolvePopulation(pop)
    print(f"{i/100} %")

route_x = np.array([])
route_y = np.array([])
route_z = np.array([])

bestRoute = pop.getFittest()
for location in bestRoute:
    route_x = np.append(route_x,location.x)
    route_y = np.append(route_y,location.y)
    route_z = np.append(route_z,location.altitude)

route_x = np.append(route_x,route_x[0])
route_y = np.append(route_y,route_y[0])
route_z = np.append(route_z,route_z[0])

print(pop.getFittest())
plt.plot(route_x,route_y,route_z,"-yo",markersize=2)
fig.tight_layout()
plt.show()