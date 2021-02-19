#Cam script
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class camera:
    def __init__(self, focal_length, sensor_dimensions, aspect_ratio, resolution):
        self.focal_length = focal_length
        self.sensor_width = sensor_dimensions[0]
        self.sensor_height = sensor_dimensions[1]
        self.resolution = resolution
        self.aspect_ratio = aspect_ratio
        self.image_height = math.sqrt(resolution*10**6/(aspect_ratio[0]/aspect_ratio[1]))   # Pixels
        self.image_width = self.image_height*(aspect_ratio[0]/aspect_ratio[1])              # Pixels
    
def getGroundCovered(cam,altitude):
    """
    Get the ground coverage in the x and y direction in metres
    """
    photo_x = (cam.sensor_width*10**-3)*altitude/(cam.focal_length*10**-3)
    photo_y = (cam.sensor_height*10**-3)*altitude/(cam.focal_length*10**-3)
    photo_size = (photo_x,photo_y)
    return photo_size

def getTimeBetweenPhotos(cam, uav_vel, forward_overlap,altitude):
    """
    Get the taken between photos for a uav with a known velocity
    """
    groundCovered = getGroundCovered(cam,altitude)
    overlap_distance = forward_overlap*groundCovered[1]
    time = overlap_distance/uav_vel
    return time

def getUavSpeed(cam, time_between_photos,forward_overlap,side_overlap,altitude):
    """
    Get the uav speed required to take photos at a certain time interval
    """
    groundCovered = getGroundCovered(cam,altitude)
    overlap_distance = forward_overlap*groundCovered[1]
    uav_speed = overlap_distance/time_between_photos
    return uav_speed
    
def getRequiredAltitude(cam,coverage_x): #Coverage of a image
    """
    Get the required altitude to obtain certain ground coverage
    """
    altitude = (cam.focal_length*10**-3)/((cam.sensor_width*10**-3)/coverage_x)
    return altitude

def getRequiredAltitudePx(cam,coverage_per_px): #Coverage of a pixel
    """
    Get the required altitude for a known pixel to distance ratio
    """
    altitude = getRequiredAltitude(cam,(coverage_per_px*10**-3)*cam.image_width)
    return altitude

def getNumberOfPasses(cam, altitude, side_overlap,length_of_roi):
    """
    Get the number of passes required for complete coverage of an area with known overlap
    """
    groundCovered = getGroundCovered(cam,altitude)
    pass_width = side_overlap*groundCovered[0]
    passes = length_of_roi/(side_overlap*groundCovered[0])
    return passes,pass_width

def truncate(val):
    newval = round(math.trunc(val*100))
    print(f"{val} -> {newval/100}")
    return newval/100

focal_length = 3.6
altitude = 120
sensor_width = 6.3
sensor_height = 4.72
sensor_dimensions = (sensor_width,sensor_height)
roi_points = {}

resolution = 12
aspect_ratio = (4,3) 

cam = camera(focal_length,sensor_dimensions, aspect_ratio,resolution)

side_overlap = 0.6
forward_overlap = 0.6

uav_vel = 10
uav_pos = (0,0)

#print(getGroundCovered(cam,altitude))
#print(getTimeBetweenPhotos(cam,uav_vel,forward_overlap,side_overlap,altitude))
#print(getUavSpeed(cam,time_between_photos,forward_overlap,side_overlap,altitude))
print(getRequiredAltitude(cam,200))
print(getRequiredAltitudePx(cam,50))

length_of_roi = 1000

print(getNumberOfPasses(cam,altitude,side_overlap,length_of_roi))

t = 0
step_size = 1

groundCovered = getGroundCovered(cam,altitude)
time_for_photo = getTimeBetweenPhotos(cam,uav_vel,forward_overlap,altitude)
passes, pass_width = getNumberOfPasses(cam, altitude, side_overlap,length_of_roi)

i=0
while t < 100000:
    if t%33000==0:
        uav_pos = (0,i*pass_width)
        i+=1
    else:
        uav_pos = (uav_pos[0] + uav_vel*(step_size/500),uav_pos[1])
    #print(uav_pos)

    if t%(time_for_photo*1000)==0:
        print(f"Rect xy = {(uav_pos[0]-groundCovered[1]/2, uav_pos[1]+groundCovered[0]/2)}")
        rectangle = plt.Rectangle((uav_pos[0]-groundCovered[1]/2, uav_pos[1]-groundCovered[0]/2), groundCovered[1], groundCovered[0], angle=0, fill=False)
        plt.gca().add_patch(rectangle)
    plt.plot(uav_pos[0],uav_pos[1],'-ro',markersize=1)
    t+=step_size*2
plt.show()