# Creating spirals

import math
import numpy as np

class Spiral:
    def __init__(self,start,end,height,radius,pitch_angle,rotations):
        self.start = start
        self.end = end
        self.length = math.pi * radius * 2 * rotations
        self.length_3d = math.sqrt((math.pi * radius * 2 * rotations)**2 + height*height)
        self.height = height
        self.radius = radius
        self.pitch_angle = pitch_angle
        self.rotations = rotations
        self.energy = None

    def get_energy(self,uav_mass):
        if self.energy is None:
            self.energy = self.get_length3d() * uav_mass * 9.81 * self.height
        return self.energy

    def get_length(self):
        return self.length
        
    def get_length3d(self):
        return self.length_3d

def create_spiral(start,height,min_turn_rad, max_incline_angle):
    circumference = 2 * math.pi * min_turn_rad
    max_height = math.tan(math.radians(max_incline_angle)) * circumference
    number_of_rotations = height/max_height
    pitch_angle = max_incline_angle
    if number_of_rotations % 1 > 0:
        number_of_rotations = int(number_of_rotations + 1) # Truncate the decimals
        height_per_rotation = height/number_of_rotations
        pitch_angle = math.atan(height_per_rotation/circumference)

    length = circumference*number_of_rotations
    end = [start[0],start[1],start[2]+height,start[3]]
    spiral = Spiral(start,end,height,min_turn_rad,pitch_angle,number_of_rotations)
    return spiral


def get_coords(points, angle,center_point,height,radius):
    point = [center_point[0] + radius*math.cos(angle),center_point[1] + radius*math.sin(angle),height]
    points.append(point)
    return points
    

def sample_spiral(spiral, step_size):
    points = []
    circumference = 2*math.pi*spiral.radius
    ratio = step_size/circumference
    step_angle = ratio*2*math.pi

    #heading_grad = math.tan(spiral.start[3])
    tangent_angle = spiral.start[3] + math.pi/2
    center_point = [spiral.start[0] + spiral.radius* math.cos(tangent_angle),
                    spiral.start[1] + spiral.radius*math.sin(tangent_angle)]

    start_angle = math.atan2(spiral.start[1]-center_point[1],spiral.start[0]-center_point[0])

    x = 0
    while x < spiral.get_length():
        height = spiral.start[2] + x*math.tan(spiral.pitch_angle)
        angle = start_angle + step_angle*x
        points = get_coords(points, angle,center_point,height,spiral.radius)
        x += step_size

    return points

if '__main__' == __name__:
    """
    Test cases for spiral class
    """
    import matplotlib.pyplot as plt

    # Setup the figure for visualising the path
    fig = plt.figure(num=1,clear=True,figsize=(12,8))
    ax = fig.add_subplot(1,1,1,projection='3d')
    ax.set(title="Spiral generated",xlabel='x', ylabel='y', zlabel='z = Height (m)')
    #ax.set_zlim(0,150)

    ax.set_aspect(aspect='auto')
    fig.tight_layout()


    start = (0,0,10,math.pi/6)
    height = 11.0070180170810870108
    max_incline_angle = 30
    min_turn_rad = 100

    spiral = create_spiral(start,height,min_turn_rad,max_incline_angle)

    step_size = 1
    spiral_points = sample_spiral(spiral, step_size)
    points_array = np.array(spiral_points)

    plt.plot(start[0],start[1],start[2],'-ro')
    plt.plot(spiral.end[0],spiral.end[1],spiral.end[2],'-ro')
    plt.plot(points_array[:,0],points_array[:,1],points_array[:,2],'-bo',markersize=2)
    plt.show()

    print(spiral.length)