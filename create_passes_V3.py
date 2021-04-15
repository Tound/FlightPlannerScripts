# Create Passes
import math
import numpy as np
import shapely.geometry as sg
from shapely import affinity
from dubins_3D import *
from scipy.interpolate import griddata
from Image_Classes_V3 import *

import matplotlib.pyplot as plt

import time

G = 9.81

class Edge:
    """
    Edge class used to test whether or not edges intercept
    parameters:
        x1  - x coordinate of start point
        y1  - y coordinate of start point
        x2  - x coordinate of end point
        y2  - y coordinate of end point
    A linestring is produced from the shapely.geometry package to represent an edge
    """
    def __init__(self,x1,y1,x2,y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.edge = sg.LineString([(x1,y1),(x2,y2)])

    def getEdge(self):
        return self.edge

def getAltitudeProfile(pass_length,terrain,uav_altitude,u,start_v,wind_angle):
    """
    Obtain altitude data for entire pass across generated terrain
    """
    altitude_profile = []
    v = start_v
    for k in range(0,round(pass_length)):
        coord = convertCoords([[u,v]],wind_angle,'xy')
        x = coord[0][0]
        y = coord[0][1]
        x_points = [int(x),int(x),int(x)+1,int(x)+1]
        y_points = [int(y),int(y)+1,int(y)+1,int(y)]
        z_points = [terrain[int(y)][int(x)],terrain[int(y)+1][int(x)],
                    terrain[int(y)+1][int(x)+1],terrain[int(y)][int(x)+1]]

        # For created terrain ONLY
        z = griddata((x_points,y_points),z_points,(x,y))    # Interpolate        
        altitude = z + uav_altitude

        altitude_profile.append(altitude)
        v +=1
    return altitude_profile

def getDistance(point1,point2):
    """
    Get the distance between two points in 2D
    parameters:
        point1 - First 2D coordinate
        point2 - Second 2D coordinate
    returns:
        Distance between points using pythagorus
    """
    dx = point2[0]-point1[0]
    dy = point2[1]-point1[1]
    return math.sqrt(dy*dy + dx*dx)

def convertCoords(vertices,angle,coord_system):
    """
    Performs coordinate transformatin between uv <=> xy
    Using   [u] = [ cos(theta) + sin(theta)][x]
            [v]   [-sin(theta) + cos(theta)][y]
    """
    theta = angle
    new_coords = []
    # Matrix multiplication?
    if coord_system == 'uv':
        for vertex in vertices:
            u = vertex[0]*math.cos(theta) + vertex[1]*math.sin(theta)
            v = -vertex[0]*math.sin(theta) + vertex[1]*math.cos(theta)
            new_coord = [u,v]
            new_coords.append(new_coord)
    elif coord_system == 'xy':
        scaler = 1/(math.cos(theta) * math.cos(theta) + math.sin(theta) * math.sin(theta))
        for vertex in vertices:
            x = scaler*(vertex[0]*math.cos(theta) - vertex[1]*math.sin(theta))
            y = scaler*(vertex[0]*math.sin(theta) + vertex[1]*math.cos(theta))
            new_coord = [x,y]
            new_coords.append(new_coord)
    else:
        print("Unknown coord system - Choose either 'xy' or 'uv'")
    return new_coords

def createTerraces(u,v,altitude_profile,wind_angle,pass_length,image_passes,max_alt_diff,min_terrace_len):
    """
    Splits pass into terraces
    """
    # Create terraces from pass
    current_terrace = []        # Initilise current terrace points as empty
    average_alt = 0             # Store the average altitude
    if len(altitude_profile) > 0: # If there is some altitude data for this pass
        max_altitude = np.max(altitude_profile)
        min_altitude = np.min(altitude_profile)
        if max_altitude - min_altitude < max_alt_diff:  # If the entire pass is within the altitude limits
            # Find the mean altitude
            for i in range(0,len(altitude_profile)):
                average_alt += altitude_profile[i]
            average_alt = average_alt/len(altitude_profile)
            terrace_start = (u,v,average_alt)
            terrace_end = (u,v+pass_length,average_alt)

            coords = convertCoords([[terrace_start[0],terrace_start[1]]],wind_angle,'xy')
            terrace_start = (coords[0][0],coords[0][1],average_alt)

            coords = convertCoords([[terrace_end[0],terrace_end[1]]],wind_angle,'xy')
            terrace_end = (coords[0][0],coords[0][1],average_alt)

            image_passes.append(Image_Pass(terrace_start,terrace_end,average_alt,wind_angle))
        else:
            # Calculate terraces
            index = 0

            current_altitude = 0
            current_min_altitude = 0
            current_max_altitude = 0
            while index < len(altitude_profile):
                if index + 3 > len(altitude_profile):
                    if current_altitude == 0:
                        for alt in range(0,2):
                            current_altitude += altitude_profile[index+alt]
                        current_altitude = current_altitude/3
                    for val in range(0,3):
                        current_terrace.append([x,y,current_altitude])
                    # Add all to current pass
                    terrace_start = current_terrace[0]
                    terrace_end = current_terrace[len(current_terrace)-1]
                    image_passes.append(Image_Pass(terrace_start,terrace_end,current_altitude,wind_angle))
                    break

                if len(current_terrace) == 0:
                    # Look ahead to find gradient
                    grad = altitude_profile[index+1] - altitude_profile[index]
                    grad += altitude_profile[index+2] - altitude_profile[index]
                    #grad += altitude_profile[index+3] - altitude_profile[index]
                    grad = grad/2
                    coords = convertCoords([[u,v+index]],wind_angle,'xy')
                    x = coords[0][0]
                    y = coords[0][1]
                    if grad > 0:
                        current_altitude = altitude_profile[index]+max_alt_diff/2
                        current_terrace.append([x,y,current_altitude])
                    elif grad < 0:
                        current_altitude = altitude_profile[index]-max_alt_diff/2
                        current_terrace.append([x,y,current_altitude])
                    else:
                        current_altitude = altitude_profile[index]
                        current_terrace.append([x,y,current_altitude])
                    current_min_altitude = current_altitude - max_alt_diff/2
                    current_max_altitude = current_altitude + max_alt_diff/2

                else:
                    # Add to the terrace
                    if altitude_profile[index] > current_min_altitude and  altitude_profile[index] < current_max_altitude:
                        coords = convertCoords([[u,v+index]],wind_angle,'xy')
                        x = coords[0][0]
                        y = coords[0][1]
                        current_terrace.append([x,y,current_altitude])
                    elif altitude_profile[index+1] > current_min_altitude and altitude_profile[index+1] < current_max_altitude:
                        coords = convertCoords([[u,v+index]],wind_angle,'xy')
                        x = coords[0][0]
                        y = coords[0][1]
                        current_terrace.append([x,y,current_altitude])
                        index += 1
                        coords = convertCoords([[u,v+index]],wind_angle,'xy')
                        x = coords[0][0]
                        y = coords[0][1]
                        current_terrace.append([x,y,current_altitude])
                    elif altitude_profile[index+2] > current_min_altitude and altitude_profile[index+2] < current_max_altitude:
                        for val in range(0,2):
                            coords = convertCoords([[u,v+index+val]],wind_angle,'xy')
                            x = coords[0][0]
                            y = coords[0][1]
                            current_terrace.append([x,y,current_altitude])
                            index += 1
                        index -= 1
                    else:
                        if len(current_terrace) > min_terrace_len:
                            # Create new terrace
                            terrace_start = current_terrace[0]
                            terrace_end = current_terrace[len(current_terrace)-1]
                            image_passes.append(Image_Pass(terrace_start,terrace_end,current_altitude,wind_angle))
                            current_terrace = []
                            current_altitude = 0
                        else:
                            # Requires more image locations
                            print("Not long enough")
                            coords = convertCoords([[u,v+index]],wind_angle,'xy')
                            x = coords[0][0]
                            y = coords[0][1]
                            current_terrace.append([x,y,current_altitude])
                            #current_max_altitude += max_alt_diff/2
                            #current_min_altitude -+ max_alt_diff/2

                index += 1
    else:
        pass
    return image_passes

def coverage_check(heading_angle,start_u,pass_shift,coverage_width,coverage_height):
    complete_coverage = True
    # Create edges
    area_bound_point = start_u
    center1 = (start_u + coverage_width/2 + pass_shift,0)
    center2 = (start_u + coverage_width/2 + pass_shift,coverage_height)
    # Edges for first camera footprint
    footprint1 = sg.Polygon([(center1[0] - coverage_width/2,center1[1] + coverage_height/2),
                            (center1[0] + coverage_width/2,center1[1] + coverage_height/2),
                            (center1[0] + coverage_width/2,center1[1] - coverage_height/2),
                            (center1[0] - coverage_width/2,center1[1] - coverage_height/2)])
    # Edges for second camera footprint
    footprint2 = sg.Polygon([(center2[0] - coverage_width/2,center2[1] + coverage_height/2),
                            (center2[0] + coverage_width/2,center2[1] + coverage_height/2),
                            (center2[0] + coverage_width/2,center2[1] - coverage_height/2),
                            (center2[0] - coverage_width/2,center2[1] - coverage_height/2)])
    # Rotate footprints accordingly
    rotated_footprint1 = affinity.rotate(footprint1,heading_angle,center1,use_radians=True)
    rotated_footprint2 = affinity.rotate(footprint2,heading_angle,center2,use_radians=True)

    # Get intersect of footprints
    intersection_points = rotated_footprint1.intersection(rotated_footprint2)

    # If coverage is complete
    # Get vertices
    # Sutherland-Hodgman for zero overlap
    plt.plot(*rotated_footprint1.exterior.xy,'-ro')
    plt.plot(*rotated_footprint2.exterior.xy,'-bo')
    plt.plot(*intersection_points.exterior.xy,'-go')
    plt.plot([area_bound_point,area_bound_point],[-coverage_height,2*coverage_height],'-yo')

    plt.title(f'Checking for complete coverage after wind compensation\nHeading angle of {round(math.degrees(heading_angle),2)} degrees')
    plt.legend(['Footprint 1','Footprint 2','Intersection area','Area bounds'],loc=1)
    plt.xlabel('u')
    plt.ylabel('v')

    #plt.axis('equal')
    plt.xlim([area_bound_point - coverage_width/2, area_bound_point+1.5*coverage_width])


    plt.show()
    u,v = intersection_points.exterior.xy
    sorted_points = sorted(u)
    if sorted_points[0] > start_u:
        complete_coverage = False

    return complete_coverage


def createPasses(area,polygon_edges,NFZs,terrain,config):
    """
    Create passes across specified area for known terrain
    Parameters
        area    - Vertices to chosen area of interest
        NFZs    - 2D array of vertices for any NFZs
        terrain - Generated terrain with altitude data
        config  - Configuration containing settings for the flight
    Returns
        image_passes - List of created passes
    """
    camera = config.camera
    wind_angle = config.wind[1]

    image_passes = []   # Initialise list of image_passes as empty

    # In case of a windless environment
    if config.wind[0] == 0:     # If the wind has no magnitude
        # Find the largest edge
        length = 0
        largest_edge = None
        for edge in polygon_edges:  # Cycle through all edges
            edge_length = edge.getEdge().length
            if edge_length > length:            # If the edge length is larger than the current largest edge length
                length = edge_length            # Store the length
                largest_edge = edge.getEdge()   # Store the edge
        coords = largest_edge.coords
        dx = coords[1][0] - coords[0][0]
        dy = coords[1][1] - coords[0][1]
        wind_angle = math.atan2(dy,dx) + math.pi/2  # Set the wind angle to the angle of the largest edge
        # Ensure that the wind angle is between the range
        if wind_angle >= 2*math.pi:
            wind_angle -= 2*math.pi
        elif wind_angle < 0:
            wind_angle += 2*math.pi
        print(f"New wind angle: {wind_angle}")


    # Update coordinate system
    new_area_coords = convertCoords(area,wind_angle,'uv')
    new_NFZs = []
    for NFZ in NFZs:
        new_NFZ_coords = convertCoords(NFZ,wind_angle,'uv')
        new_NFZs.append(new_NFZ_coords)

    # Find footprint size
    coverage_width = config.ground_sample_distance * camera.image_x
    coverage_height = config.ground_sample_distance * camera.image_y

    uav_altitude = coverage_width *camera.focal_length/camera.sensor_x
    max_uav_alt = (camera.image_x * (config.ground_sample_distance + 0.0015)) * camera.focal_length/camera.sensor_x
    min_uav_alt = (camera.image_x * (config.ground_sample_distance - 0.0015)) * camera.focal_length/camera.sensor_x

    distance_between_photos_width = coverage_width - coverage_width*config.side_overlap

    if coverage_height/2 > config.uav.min_turn:
        print("Warning - At this altitude the UAV will mostly likely fly into an NFZ" + 
        " as the minimum turn radius is larger than the area between the end of passes"+ 
        " and the NFZ boundary. Consider increasing altitude or the ground sample_resolution.")

    # Obtain properties about the area
    sorted_vertices = sorted(new_area_coords, key=lambda u:u[0])
    length_of_area = abs(sorted_vertices[len(sorted_vertices)-1][0] - sorted_vertices[0][0])  # Obtain the  horizontal length of the area in pixels

    np_area = np.array(new_area_coords)         # Convert the new coords to a numpy array for easier work
    max_height = np.max(np_area[:,1])           # Store highest V value of the entire area
    min_height = np.min(np_area[:,1])           # Store lowest V value of the entire area
    start_u = np.min(np_area[:,0])              # Save the left most horizontal value for the start of the area
    
    # Calculate the number of passes that will cover the area
    number_of_passes = (length_of_area-config.side_overlap*coverage_width)/distance_between_photos_width

    # If the number of passes required is not an integer, add another to ensure coverage
    if number_of_passes % 1 > 0: # If number of passes is not an integer to create complete coverage
        number_of_passes = int(number_of_passes+1)
        # Center the passes
        # Create shift value to center the passes
        remainder = length_of_area - (number_of_passes * coverage_width - (number_of_passes-1)*config.side_overlap*coverage_width) 
        pass_shift = remainder/2
    else:
        pass_shift = 0
        print("Passes are integer value and therefore there is no overlap and do no require shifting")

    # Check if wind is present, if so the heading angle will shift and so will the coverage
    if config.wind[0] > 0:
        if not coverage_check(config.uav.heading_angle,start_u,pass_shift,coverage_width,coverage_height):   # Check if coverage is still complete
            print("Coverage is no longer complete, adding another pass")
            # Check if another one is required
            # If another pass is required, add another then recenter the passes
            number_of_passes += 1
            remainder = length_of_area - (number_of_passes * coverage_width - (number_of_passes-1)*config.side_overlap*coverage_width) 
            pass_shift = remainder/2
        else:
            print("Another pass is not required due to the current configuration allowing complete coverage")
    else:
        print("There is no wind and therefore no heading angle compensation")

    polygon_edges = []
    NFZ_edges = []

    # Create all new polygon edges
    for i in range(0,len(new_area_coords)):
        polygon_edges.append(Edge(new_area_coords[i-1][0],new_area_coords[i-1][1],
                                    new_area_coords[i][0],new_area_coords[i][1]))
    
    # Create all new NFZ edges
    for NFZ_coords in new_NFZs:
        for i in range(0,len(NFZ_coords)):
            NFZ_edges.append(Edge(NFZ_coords[i-1][0],NFZ_coords[i-1][1],
                                    NFZ_coords[i][0],NFZ_coords[i][1]))

    # Set terrace properties
    # Min terrace length
    min_terrace_length = 20
    max_alt_diff = max_uav_alt - min_uav_alt

    # Shift passes to allow for even distribution
    u = start_u + coverage_width/2 + pass_shift
    for i in range(0,number_of_passes):        # Cycle through all full-length passes across entirety of area
        # Find points where passes intersect with ROI

        intersection_points = []    #Points of intersection for each pass
        pass_edge = sg.LineString([(u,min_height-1),(u,max_height+1)])

        max_intersect = [-math.inf,-math.inf]
        min_intersect = [math.inf,math.inf]

        for edge in polygon_edges:
            intersection = edge.getEdge().intersection(pass_edge)
            if intersection.is_empty:
                continue
            if type(intersection) == sg.Point:
                 if intersection.y >= max_intersect[1]:               # Even though we are using uv axis, shapely requires xy
                     max_intersect = [intersection.x,intersection.y]
                 if intersection.y <= min_intersect[1]:
                     min_intersect = [intersection.x,intersection.y]
                #intersection_points.append([intersection.x,intersection.y])
            elif type(intersection) == sg.LineString:
                for point in intersection.coords:
                    if point[1] >= max_intersect[1]:
                        max_intersect = [point[0],point[1]]
                    if point[1] <= min_intersect[1]:
                        min_intersect = [point[0],point[1]]
                    #intersection_points.append([point[0],point[1]])
        intersection_points.extend([min_intersect,max_intersect])



        # Find where passes intersect with NFZs
        for edge in NFZ_edges:        # Cycle through all NFZs
            intersection = edge.getEdge().intersection(pass_edge)
            if intersection.is_empty:
                continue
            else:
                intersection_points.append([intersection.x,intersection.y])

        # Remove duplicates?

        points_on_pass = sorted(intersection_points,key=lambda point:point[1])   # List vertically
        subpasses = len(points_on_pass)/2

        print(points_on_pass)
        print(subpasses)
        for j in range(0,int(subpasses)):   # Split full length passes into sub passes if obstacles are found
            start = points_on_pass[j*2]
            end = points_on_pass[j*2 + 1]
            pass_length = getDistance(start,end)-coverage_height
            print(f"Start: {start}, End: {end}, Length: {pass_length}")
            if pass_length <= 0:
                print("Pass length is too small")
                continue

            start_v = start[1] + coverage_height/2  # Shift pass up by half the size of an image height

            # Get altitude data
            altitude_profile = getAltitudeProfile(pass_length,terrain,uav_altitude,u,start_v,wind_angle)

            v = start_v
            image_passes = createTerraces(u,v,altitude_profile,wind_angle,pass_length,image_passes,max_alt_diff,min_terrace_length)
        u += distance_between_photos_width          # Increase U value on each loop

    print(f"length = {length_of_area}")
    print(f"Footprint of image: {coverage_width}x{coverage_height}")
    print(f"Distance between passes: {round(distance_between_photos_width,2)} m")
    print(f"number_of_passes = {number_of_passes}")
    print(f"Min altitude: {round(min_uav_alt,2)} m\nMax altitude: {round(max_uav_alt,2)}m"
            +f"\nDesired altitude: {round(uav_altitude,2)}")

    return image_passes