"""
Script used to create and sample spirals for the creation of
efficient flight paths.
Created by Thomas Pound
Last updated 30/4/21
"""

import math
import numpy as np

class Spiral:
    """
    Spiral class used to store all parameters that make a required spiral
    """
    def __init__(self,start,end,height,radius,pitch_angle,rotations):
        self.start = start                                          # Start point as a tuple
        self.end = end                                              # End point as a tuple
        self.length = math.pi * radius * 2 * rotations              # Calculate the 2D length of the spiral
        self.length_3d = math.sqrt(self.length*self.length + height*height)     # Calculate the 3D length of the spiral
        self.height = height                                        # Height of spiral
        self.radius = radius                                        # Radius of spiral
        self.pitch_angle = pitch_angle                              # Pitch angle of the spiral
        self.rotations = rotations                                  # Number of complete rotations in the spiral
        self.energy = None                                          # Relative energy required to complete the spiral

    def get_energy(self,uav_mass):
        """
        Get the relative energy required to complete the spiral
        Params:
            uav_mass - Mass of the UAV in Kg 
        Return:
            Energy
        """
        if self.energy is None:
            self.energy = self.get_length3d() * uav_mass * 9.81 * self.height
        return self.energy

    def get_length(self):
        return self.length
        
    def get_length3d(self):
        return self.length_3d

def create_spiral(start,height,min_turn_rad, max_incline_angle):
    """
    Create spiral object used to create helical path
    """
    circumference = 2 * math.pi * min_turn_rad                                      # Calculate the circumference of a single 
    height_per_rotation = math.tan(math.radians(max_incline_angle)) * circumference # Calculate the height of each rotation
    number_of_rotations = height/height_per_rotation                # Calculate the number of full rotations required
    pitch_angle = max_incline_angle
    if number_of_rotations % 1 > 0:                                 # If the number of rotations is not an integer
        number_of_rotations = int(number_of_rotations + 1)          # Truncate the decimals
        height_per_rotation = height/number_of_rotations            # Update new height
        pitch_angle = math.atan(height_per_rotation/circumference)  # Update the pitch angle

    length = circumference*number_of_rotations                      # Calculate the entire length of the spiral path
    end = [start[0],start[1],start[2]+height,start[3]]              # Set the end path
    spiral = Spiral(start,end,height,min_turn_rad,pitch_angle,number_of_rotations)  # Create new spiral object
    return spiral


def get_coords(points,angle,center_point,height,radius):
    """
    Function used to print the sampled coordinates of the spiral
    Parameters
        points - Array of points on sapled spiral
        angle - Current heading angle of the points
        center_point - The center point of the spiral
        height - Current height of the sample point
        radius - Radius of the spiral
    Return
        points - The appended points array
    """
    # Calculate the sampled point of the spiral
    point = [center_point[0] + radius*math.cos(angle),
            center_point[1] + radius*math.sin(angle),height]
    points.append(point)    # Append the new point to the list of sampled points
    return points
    

def sample_spiral(spiral, step_size):
    """
    Function used to sample a spiral object
    Parameters
        spiral - A constructed spiral to be sampled
        step_size - The desired step size in metres
    Returns
        points - The list of sampled points
    """
    points = []                             # Initialise list of sampled points as empty
    circumference = 2*math.pi*spiral.radius # Calculate the circumference of a flat spiral
    ratio = step_size/circumference         # Calculate how large a step size is compared to the circumference
    step_angle = ratio*2*math.pi            # Calculate the difference in angle for each step

    tangent_angle = spiral.start[3] + math.pi/2             # Calculate the angle of the tangent
    # Calculate the center point of the spiral
    center_point = [spiral.start[0] + spiral.radius* math.cos(tangent_angle),
                    spiral.start[1] + spiral.radius*math.sin(tangent_angle)]

    # Calculate the angle of the starting point
    start_angle = math.atan2(spiral.start[1]-center_point[1],spiral.start[0]-center_point[0])

    x = 0                           # Set the total length travelled to 0
    while x < spiral.get_length():  # While the distance sampled is less than the length of the spiral
        height = spiral.start[2] + x*math.tan(spiral.pitch_angle)   # Calculate the current height of the spiral
        angle = start_angle + step_angle*x                          # Calculate the current heading angle of the spiral
        points = get_coords(points, angle,center_point,height,spiral.radius)    # Add the sampled point to the list of points
        x += step_size                          # Add a step size to the distance travelled

    return points

if '__main__' == __name__:
    """
    Test cases for spiral class
    """
    import matplotlib.pyplot as plt

    # Test parameters
    start = (0,0,10,math.pi/2)      # Start location
    height = 100                    # Desired height
    max_incline_angle = 20          # Max incline angle in degrees
    min_turn_rad = 20               # Radius of the spiral

    spiral = create_spiral(start,height,min_turn_rad,max_incline_angle) # Create spiral object

    step_size = 1                                       # Desired sample step size
    spiral_points = sample_spiral(spiral, step_size)    # Get sampled points
    points_array = np.array(spiral_points)              # Convert to numpy array for drawing on the graph

    # Setup the figure for visualising the path
    fig = plt.figure(num=1,clear=True,figsize=(12,8))
    ax = fig.add_subplot(1,1,1,projection='3d')
    ax.set(title=f"Spiral generated\nRadius: {min_turn_rad} m, Pitch angle: {round(math.degrees(spiral.pitch_angle),2)} degs, "
            +f"Spiral length: {round(spiral.length_3d,2)} m",xlabel='x', ylabel='y', zlabel='z = Height (m)')
    #ax.set_zlim(0,150)

    ax.set_aspect(aspect='auto')
    fig.tight_layout()

    # Plot the sampled spiral
    plt.plot(start[0],start[1],start[2],'-ro')
    plt.plot(spiral.end[0],spiral.end[1],spiral.end[2],'-ro')
    plt.plot(points_array[:,0],points_array[:,1],points_array[:,2],'-bo',markersize=2)
    plt.show()

    print(spiral.length)