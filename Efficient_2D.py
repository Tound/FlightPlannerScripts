#Efficient 2D
import matplotlib.pyplot as plt
import matplotlib.path as path
import matplotlib.patches as patches
import matplotlib.transforms as t
import numpy as np
import math
from tsp import *

imaging_passes  = []
all_image_locations = []

#fig = plt.figure()
#ax = fig.add_subplot(111)

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

def calculate_cost():
    cost = 10
    return cost


# UAV settings
min_turn = 2 #m

# Wind settings
wind = (10,math.radians(-45)) #Polar coords (Mag, degrees)

# Camera settings
side_overlap = 0#0.5          # Percentage
forward_overlap =0#0.4       # Percentage
sensor_x = 5.62    *10**-3  # mm
sensor_y = 7.4     *10**-3  # mm
focal_length = 3.6 *10**-3  # mm
aspect_ratio = (4,3)        # x:y
cam_resolution = 12         # MP
image_x = 4000              # px
image_y = 3000              # px

# Flight settings`
coverage_resolution = 0.1 #0.02  # m/px

# Start/end location
start_pos = (0,0)

# AOI
polygon = [[200,1000],[800,2500],[2000,3000],[2500,2500],[3000,1500],[3000,100],[1500,-800],[500,-500]]
polygon.append(polygon[0])

NFZ = [[],[]]

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
    #plt.plot(x_points,pass_grad*x_points + y_shift,'o', color= 'red',markersize=1)


# Draw the polygon

poly_x = np.array([])
poly_y = np.array([])
for vertex in polygon:
    poly_x = np.append(poly_x,vertex[0])
    poly_y = np.append(poly_y,vertex[1])

plt.plot(poly_x,poly_y,'-bo')
#plt.plot(min_point[0],min_point[1],'go')
#plt.plot(max_point[0],max_point[1],'ro')





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
        altitude = uav_altitude
        image_loc(x,y,altitude) # Create new image location
        image_locations.append(image_loc) # Add object to list
        # Add to long list of all image locations
        all_image_locations.append((x,y))   # 2D in this case
        plt.plot(x,y,'bo',markersize=3)
        #ts = ax.transData
        #tr = t.Affine2D().rotate_deg_around(x,y,math.degrees(-pass_grad))
        #tr = tr + ts
        #rect = patches.Rectangle((x-coverage_x/2,y-coverage_y/2),coverage_y,coverage_x,angle=0,fill=None,transform=tr)
        #plt.gca().add_patch(rect)
    imaging_pass.add_img_locs(image_locations)

#print(all_image_locations)

all_image_locations.insert(0,start_pos)

#TSP
i = 0
tourmanager = TourManager()
for loc in all_image_locations:
    location = City(loc[0],loc[1])
    tourmanager.addCity(location)

# Initialize population
pop = Population(tourmanager, len(all_image_locations), True)
print( "Initial distance: " + str(pop.getFittest().getDistance()))

# Evolve population for 50 generations
ga = GA(tourmanager)
pop = ga.evolvePopulation(pop)
for i in range(0, 10000):
    pop = ga.evolvePopulation(pop)

tour_x = np.array([])
tour_y = np.array([])
# Print final results
print( "Finished")
print( "Final distance: " + str(pop.getFittest().getDistance()))
print( "Solution:")
#print( pop.getFittest())
string = str(pop.getFittest())
string = string[1:-3]
string = string.split("),(")
for word in string:
    word = word.split(',')
    print(word)
    tour_x = np.append(tour_x,float(word[0]))
    tour_y = np.append(tour_y,float(word[1]))

plt.plot(tour_x,tour_y,'-yo',markersize = 2)

plt.show()

#while i < 10000:

#    i +=1