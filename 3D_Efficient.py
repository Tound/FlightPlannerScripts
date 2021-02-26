#3D_EFFICIENT
#3D Coverage
import matplotlib.pyplot as plt
import matplotlib.path as path
import matplotlib.patches as patches
import math
import numpy as np
import random
from opensimplex import OpenSimplex
from Modified_Dubins_TSP import *
import shapely.geometry

########################
# CREATE TERRAIN
########################
height = 750
width = 750
freq = 2

gen = OpenSimplex()#seed=random.randint(0,100))
def noise(nx, ny):
    # Rescale from -1.0:+1.0 to 0.0:1.0
    return gen.noise2d(nx, ny) / 2.0 + 0.5

value = np.zeros((width,height))
for y in range(height):
    for x in range(width):
        nx = x/width - 0.5
        ny = y/height - 0.5
        elevation = 20*noise(freq*nx, freq*ny)
        value[y][x] =  20*math.pow(elevation,0.5)


########################
# Flight setup
########################
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
min_turn = 25 #m
max_incline_grad = 30 #degs
uav_mass = 18 # Kg
uav_speed = 50

# Wind settings
wind = (10,math.radians(-45)) #Polar coords (Mag, degrees)

if math.degrees(wind[1]) > 360:
    degs = math.degrees(wind[1]) -360
else:
    degs = math.degrees(wind[1])

if degs < 0:
    wind_bearing = -degs+90
else:
    wind_bearing = -degs + 90
if wind_bearing < 0:wind_bearing = 360+wind_bearing
elif wind_bearing <= 360: wind_bearing -= 360



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

# Add lens FOV

# Flight settings
coverage_resolution = 0.05  # m/px

# Start/end location
#start_pos = (0,0)

# Viablility checks
if uav_speed <= wind[0]:
    # Check air density?
    print("UAV speed is too slow compared to wind speed - Unsafe conditions")
    exit(1)

# AOI
polygon = [[100,100],[100,650],[650,650],[650,100]]
polygon.append(polygon[0])

start_loc = [20,730,value[20][730]]

NFZ = [[200,450],[450,450],[450,200],[200,200]]
NFZ.append(NFZ[0])

NFZs = [NFZ]

NFZ_Poly = shapely.geometry.Polygon(NFZ)
NFZ_Polys = [NFZ_Poly]

coverage_x = coverage_resolution * image_x
coverage_y = coverage_resolution * image_y

uav_altitude = coverage_x *focal_length/sensor_x      # Above ground

distance_between_photos_x = coverage_x - coverage_x*side_overlap
distance_between_photos_y = coverage_y - coverage_y*forward_overlap

# Create image positions
# Wind as uniform plane
wind_dx = math.cos(wind[1])
wind_dy = math.sin(wind[1])
wind_grad = wind_dy/wind_dx
pass_grad = -1/wind_grad

pass_angle = math.atan(pass_grad)


print(pass_grad,wind_grad)
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

# Create paths

polygon_path = path.Path(polygon)

NFZ_Paths = []
for NFZ in NFZs:
    NFZ_Paths.append(path.Path(NFZ))



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

# SETUP GRAPH

fig = plt.figure(num=1,clear=True,figsize=(12,8))
ax = fig.add_subplot(1,1,1,projection='3d')

(x,y) = np.meshgrid(np.arange(0,width,1),np.arange(0,height,1))
ax.plot_surface(x, y, value,cmap='terrain')
ax.set(title=f'Terrain Generated \n Wind Direction: {math.degrees(wind[1])} degrees \n Bearing: {wind_bearing}',xlabel='x', ylabel='y', zlabel='z = Height (m)')

# Draw the polygon

poly_x = np.array([])
poly_y = np.array([])
for vertex in polygon:
    poly_x = np.append(poly_x,vertex[0])
    poly_y = np.append(poly_y,vertex[1])

plt.plot(poly_x,poly_y,'-bo')

# Draw NFZs

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



def inNFZs(NFZ_Paths):
    for nfz_path in NFZ_Paths:
        if nfz_path.contains_point((x,y)):
            return True
    return False
    
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

        if inNFZs(NFZ_Paths):
            continue
        altitude = uav_altitude + value[int(y)][int(x)]
        image_loc(x,y,altitude)                     # Create new image location
        image_locations.append(image_loc)           # Add object to list
        # Add to long list of all image locations
        all_image_locations.append((x,y,altitude))
        plt.plot(x,y,altitude,'-bo',markersize=3)
    imaging_pass.add_img_locs(image_locations)      # Add image locations to imaging pass

#####################
# TSP 
#####################

all_image_locations.insert(0,start_loc)

routemanager = RouteManager()
routemanager.setParams(pass_angle,wind[1],min_turn,uav_mass,NFZ_Polys, max_incline_grad)
for location in all_image_locations:
    image_location = ImageLocation(location[0],location[1],location[2])
    routemanager.addImageLocation(image_location)

# Initialize population
pop = Population(routemanager, 100, True)#routemanager.numberOfLocations(), True)
print( "Initial distance: " + str(pop.getFittest().getEnergy()))

# Evolve population for 50 generations
ga = GA(routemanager,0.015,20)
pop = ga.evolvePopulation(pop)
for i in range(0, 1000):
    pop = ga.evolvePopulation(pop)
    print(f"{i/10} %")

route_x = np.array([])
route_y = np.array([])
route_z = np.array([])

dubins_x = np.array([])
dubins_y = np.array([])
dubins_z = np.array([])

bestRoute = pop.getFittest()
for location in bestRoute:
    route_x = np.append(route_x,location.x)
    route_y = np.append(route_y,location.y)
    route_z = np.append(route_z,location.altitude)

route_x = np.append(route_x,route_x[0])
route_y = np.append(route_y,route_y[0])
route_z = np.append(route_z,route_z[0])

dpaths = bestRoute.getDPaths()
index = 0
# Could use np.where
for location in bestRoute.getRoute():
    if start_loc[0] == location.x and start_loc[1] == location.y:
        start_index = index
        break
    index+=1

route = np.array(bestRoute.getRoute())
route = np.roll(route,-start_index)         # Shift start location to start of array

# Show the points that are made from the dubins paths
for dpath in dpaths:
    points = dubins_path_sample_many(dpath,10)
    for point in points[:-1]:
        plt.plot(point[0],point[1],point[2],'go',markersize=1,zorder=5)
        #plt.plot(point[0],point[1],200,'go',markersize=1,zorder = 5)
        #dubins_x = np.append(dubins_x, point[0])
        #dubins_y = np.append(dubins_y, point[1])
        #dubins_z = np.append(dubins_z,200)

plt.plot(route_x,route_y,route_z,"-yo",markersize=2,label='TSP Route',zorder=10)

ax.quiver(20,730,250,10*math.cos(wind[1]),10*math.sin(wind[1]),0,length=20,arrow_length_ratio=0.1)
ax.quiver(route_x[0],route_y[0],route_z[0],route_x[1]-route_x[0],route_y[1]-route_y[0],route_z[1]-route_z[0],arrow_length_ratio=0.1)
#plt.plot(dubins_x,dubins_y,dubins_z,"-go",markersize=1)
plt.legend()
fig.tight_layout()
plt.show()

print(f"Energy used = {1/bestRoute.getFitness()}")
print("Estimated time to complete path = ")