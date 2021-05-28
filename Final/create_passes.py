"""
Creates passes and terraces for a photogrammetry flight path
Created by Thomas Pound
Last updated 28/5/21
"""
import math
import numpy as np
import shapely.geometry as sg
from shapely import affinity
from dubins_3D import *
from scipy.interpolate import griddata
from Image_Classes import *
from camera_calculations import *

G = 9.81        # Acceleration due to gravity

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

    def __repr__(self):
        return f"([{self.x1},{self.y1}),({self.x2},{self.y2}])"

def get_altitude_profile(pass_length,terrain,uav_altitude,u,start_v,wind_angle):
    """
    Obtain altitude data for an entire pass across generated terrain
    Params:
        pass_length - Length of the pass in metres
        terrain - The 2D terrain of elevation values
        uav_altitude - The desired UAV altitude
        u - Current u coordinate
        start_v - Start v coordinate for the pass
        wind_angle - Direction of the wind in radians
    Returns:
        altitude_profile - The calculated altitude profile for the pass
    """
    altitude_profile = []
    v = start_v
    for i in range(0,round(pass_length)):               # Loop for the length of the pass
        coord = convert_coords([[u,v]],wind_angle,'xy')  # Convert coordinates back to xy coordinate system for the terrain
        x = coord[0][0]
        y = coord[0][1]
        # Take the integer points around the desired elevation point
        x_points = [int(x),int(x),int(x)+1,int(x)+1]    
        y_points = [int(y),int(y)+1,int(y)+1,int(y)]
        z_points = [terrain[int(y)][int(x)],terrain[int(y)+1][int(x)],
                    terrain[int(y)+1][int(x)+1],terrain[int(y)][int(x)+1]]

        # For created terrain ONLY
        z = griddata((x_points,y_points),z_points,(x,y))    # Interpolate terrain to find elevation inbetween points
        altitude = z + uav_altitude                         # Store the altitude above sea level

        altitude_profile.append(altitude)   # Add the altitude value to the altitude profile
        v +=1                               # Move up to the next coordinate on the pass
    return altitude_profile

def get_distance(point1,point2):
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

def convert_coords(vertices,angle,coord_system):
    """
    Performs coordinate transformatin between uv <=> xy
    Using   [u] = [ cos(theta) + sin(theta)][x]
            [v]   [-sin(theta) + cos(theta)][y]
    """
    theta = angle       # Store the angle
    new_coords = []     # Initialise an empty array to store the new coordinates
    if coord_system == 'uv':    # If the requested coordinate system is uv
        for vertex in vertices:
            u = vertex[0]*math.cos(theta) + vertex[1]*math.sin(theta)
            v = -vertex[0]*math.sin(theta) + vertex[1]*math.cos(theta)
            new_coord = [u,v]
            new_coords.append(new_coord)
    elif coord_system == 'xy':  # If the requested coordinate system is xy
        scaler = 1/(math.cos(theta) * math.cos(theta) + math.sin(theta) * math.sin(theta))
        for vertex in vertices:
            x = scaler*(vertex[0]*math.cos(theta) - vertex[1]*math.sin(theta))
            y = scaler*(vertex[0]*math.sin(theta) + vertex[1]*math.cos(theta))
            new_coord = [x,y]
            new_coords.append(new_coord)
    else:
        print("Unknown coord system - Choose either 'xy' or 'uv'")
    return new_coords

def create_terraces(u,v,altitude_profile,wind_angle,pass_length,image_passes,max_alt_diff,min_terrace_len):
    """
    Splits pass into terraces using the altitude profile
    Params:
        u - Current u value
        v - Current v value
        altitude_profile - The altitude profile for this pass
        wind_angle - The wind direction in radians
        pass_length - The length of the entire pass in metres
        image_passes - The current list of all image passes
        max_alt_diff - The maximum altitude different allowed
        min_terrace_len - The minimum length of a terrace/pass
    Returns:
        image_passes - Updated list of all image passes
    """

    lookahead = 3                                       # Initialise how many values to look ahead in altitude profile
    current_terrace = []                                # Initialise current terrace points as empty
    average_alt = 0                                     # Store the average altitude
    if len(altitude_profile) > 0:                       # If there is some altitude data for this pass
        max_altitude = np.max(altitude_profile)         # Store the maximum altitude for the pass
        min_altitude = np.min(altitude_profile)         # Store the minimum altitude for the pass
        if max_altitude - min_altitude < max_alt_diff:  # If the entire pass is within the altitude limits
            # Make entire pass into a terrace of average height

            # Find the mean altitude
            for i in range(0,len(altitude_profile)):
                average_alt += altitude_profile[i]
            average_alt = average_alt/len(altitude_profile)
            terrace_start = (u,v,average_alt)
            terrace_end = (u,v+pass_length,average_alt)

            # Convert the terrace coords back to xy and set the start and end points of the terrace
            coords = convert_coords([[terrace_start[0],terrace_start[1]]],wind_angle,'xy')
            terrace_start = (coords[0][0],coords[0][1],average_alt)

            coords = convert_coords([[terrace_end[0],terrace_end[1]]],wind_angle,'xy')
            terrace_end = (coords[0][0],coords[0][1],average_alt)

            # Add new image pass
            image_passes.append(Image_Pass(terrace_start,terrace_end,average_alt,wind_angle))
        else:
            # Calculate terraces
            index = 0

            # Initialise terrace parameters
            current_altitude = -1
            current_min_altitude = -1
            current_max_altitude = -1 

            while index < len(altitude_profile):

                if index + lookahead > len(altitude_profile):       # If unable to look ahead 3 samples (3 metres)
                    if current_altitude == -1:              # If no points are currently in the terrace (current altitude is invalid)
                        for alt in range(0,lookahead-1):              # Take average of the rest of the data
                            current_altitude += altitude_profile[index+alt]
                        current_altitude = current_altitude/3
                    
                    # Use current altitude previously calculated from the terrace
                    for val in range(0,lookahead-1):
                        coords = convert_coords([[u,v+index+val]],wind_angle,'xy')
                        current_terrace.append([coords[0][0],coords[0][1],current_altitude])  # Add points to terrace at the current altitude

                    # Add all to current pass
                    terrace_start = current_terrace[0]
                    terrace_end = current_terrace[len(current_terrace)-1]
                    image_passes.append(Image_Pass(terrace_start,terrace_end,current_altitude,wind_angle))
                    break

                if len(current_terrace) == 0:
                    # Look ahead to find gradient
                    grad = altitude_profile[index+1] - altitude_profile[index]
                    grad += altitude_profile[index+2] - altitude_profile[index]
                    grad = grad/2           # Take the average of the gradient
                    coords = convert_coords([[u,v+index]],wind_angle,'xy')
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
                    # If the altitude value is within limits
                    if altitude_profile[index] > current_min_altitude and altitude_profile[index] < current_max_altitude:
                        coords = convert_coords([[u,v+index]],wind_angle,'xy')
                        x = coords[0][0]
                        y = coords[0][1]
                        current_terrace.append([x,y,current_altitude])
                    # If the next altitude value is within limits
                    elif altitude_profile[index+1] > current_min_altitude and altitude_profile[index+1] < current_max_altitude:
                        coords = convert_coords([[u,v+index]],wind_angle,'xy')
                        x = coords[0][0]
                        y = coords[0][1]
                        current_terrace.append([x,y,current_altitude])  # Add current point
                        index += 1
                        coords = convert_coords([[u,v+index]],wind_angle,'xy')
                        x = coords[0][0]
                        y = coords[0][1]
                        current_terrace.append([x,y,current_altitude])  # Add next point
                    # If the following altitude value is within limits
                    elif altitude_profile[index+2] > current_min_altitude and altitude_profile[index+2] < current_max_altitude:
                        for val in range(0,2):
                            coords = convert_coords([[u,v+index+val]],wind_angle,'xy')
                            x = coords[0][0]
                            y = coords[0][1]
                            current_terrace.append([x,y,current_altitude])  # Add current, next and following point to the terrace
                            index += 1
                        index -= 1
                    else:
                        if len(current_terrace) > min_terrace_len:
                            # Create new terrace
                            terrace_start = current_terrace[0]
                            terrace_end = current_terrace[len(current_terrace)-1]
                            image_passes.append(Image_Pass(terrace_start,terrace_end,current_altitude,wind_angle))
                            current_terrace = []    # Initialise the array of terrace points
                            current_altitude = 0    # Initialise the terrace altitude to 0
                        else:
                            # Requires more image locations
                            print("Not long enough")
                            coords = convert_coords([[u,v+index]],wind_angle,'xy')
                            x = coords[0][0]
                            y = coords[0][1]
                            current_terrace.append([x,y,current_altitude])  # Add point to terrace
                index += 1
    else:
        pass
    return image_passes

def coverageCheck(heading_angle,start_u,pass_shift,coverage_width,coverage_height):
    """
    Ensures that the heading angle does not affect complete coverage of the area
    Params:
        heading_angle - The heading angle of the UAV in radians
        start_u - The current u value
        pass_shift - The u shift of the passes
        coverage_width - Coverage width of the image footprint in metres
        coverage_height - Coverage height of the image footprint in metres
    Returns:
        complete_coverage -  Boolean value indicating whether the coverage of the image footprints is complete
    """
    complete_coverage = True
    # Create edges
    area_bound_point = start_u
    # Find center points of 2 camera footprints that are placed on top of each other (0% overlap)
    center1 = (start_u + coverage_width/2 + pass_shift,0)
    center2 = (start_u + coverage_width/2 + pass_shift,coverage_height)
    # Create path for first camera footprint
    footprint1 = sg.Polygon([(center1[0] - coverage_width/2,center1[1] + coverage_height/2),
                            (center1[0] + coverage_width/2,center1[1] + coverage_height/2),
                            (center1[0] + coverage_width/2,center1[1] - coverage_height/2),
                            (center1[0] - coverage_width/2,center1[1] - coverage_height/2)])
    # Create path for second camera footprint
    footprint2 = sg.Polygon([(center2[0] - coverage_width/2,center2[1] + coverage_height/2),
                            (center2[0] + coverage_width/2,center2[1] + coverage_height/2),
                            (center2[0] + coverage_width/2,center2[1] - coverage_height/2),
                            (center2[0] - coverage_width/2,center2[1] - coverage_height/2)])
    # Rotate footprints accordingly
    rotated_footprint1 = affinity.rotate(footprint1,heading_angle,center1,use_radians=True)
    rotated_footprint2 = affinity.rotate(footprint2,heading_angle,center2,use_radians=True)

    # Get intersect of footprints
    intersection_points = rotated_footprint1.intersection(rotated_footprint2)

    # Get vertices
    u,v = intersection_points.exterior.xy
    sorted_points = sorted(u)       # Sort all vertices from left to right
    if sorted_points[0] > start_u:  # If the left most point is to the right side of the ROI boundary
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
    if config.wind[0] == 0:         # If the wind has no magnitude
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


    # Create new ROI coords for the updated coordinate system
    new_area_coords = convert_coords(area,wind_angle,'uv')   # Convert xy coordinates to uv system

    # Create new NFZ coords for the updated coordinate system
    new_NFZs = []
    for NFZ in NFZs:
        new_NFZ_coords = convert_coords(NFZ,wind_angle,'uv') # Convert xy coordinates to uv system
        new_NFZs.append(new_NFZ_coords)

    # Find camera footprint size
    # If GSD is available
    if config.altitude is None and config.ground_sample_distance is not None:

        # Get the coverage dimensions from the GSD
        coverage_width, coverage_height = camera.get_coverage_size_gsd(config.ground_sample_distance)
        uav_altitude = camera.get_altitude(coverage_width)

        # Get the max and min uav altitudes
        max_uav_alt = camera.get_altitude_from_gsd(config.ground_sample_distance + config.ground_sample_distance/10)
        min_uav_alt = camera.get_altitude_from_gsd(config.ground_sample_distance - config.ground_sample_distance/10)

        config.altitude = uav_altitude

    # If altitude is available
    elif config.altitude is not None and config.ground_sample_distance is None:

        # Get the coverage dimensions from the altitude
        coverage_width, coverage_height = camera.get_coverage_size_alt(config.altitude)

        uav_altitude = config.altitude

        ground_sample_distance = camera.get_gsd_from_alt(uav_altitude)    # Get GSD
        config.ground_sample_distance = ground_sample_distance
        
        # Get the max and min uav altitudes
        max_uav_alt = camera.get_altitude_from_gsd(ground_sample_distance + config.ground_sample_distance/10)
        min_uav_alt = camera.get_altitude_from_gsd(ground_sample_distance - config.ground_sample_distance/10)

    # If both have been initialised
    elif config.altitude is not None and config.ground_sample_distance is not None:
        
        # Get the coverage dimensions from the altitude
        coverage_width, coverage_height = camera.get_coverage_size_alt(config.altitude)

        # Look for conflicts
        uav_altitude = config.altitude
        ground_sample_distance = camera.get_gsd_from_alt(uav_altitude)
        if ground_sample_distance != config.ground_sample_distance: # If the values have conflicts
            print(f"Conflict with GSD, taking altitude as true. New GSD: {ground_sample_distance}")
            config.ground_sample_distance = ground_sample_distance
            
        # Get the max and min uav altitudes
        max_uav_alt = camera.get_altitude_from_gsd(ground_sample_distance + config.ground_sample_distance/10)
        min_uav_alt = camera.get_altitude_from_gsd(ground_sample_distance - config.ground_sample_distance/10)

    else:
        # If altitude and GSD are not available
        print("Requires atleast one value of altitude or gsd")

    # Calculate the distance between adjacent photos using the side overlap
    distance_between_photos_width = coverage_width - coverage_width*config.side_overlap

    # Check if UAV is likely to enter an NFZ
    if coverage_height/2 > config.uav.min_turn and len(NFZs) > 0:
        print("Warning - At this altitude the UAV will mostly likely fly into an NFZ\n " + 
        " as the minimum turn radius is larger than the area between the end of passes\n "+ 
        " and the NFZ boundary. Consider increasing altitude or the ground sample_resolution.")

    # Obtain properties about the area
    sorted_vertices = sorted(new_area_coords, key=lambda u:u[0])    # Sort the new vertices by u value
    length_of_area = abs(sorted_vertices[len(sorted_vertices)-1][0] - sorted_vertices[0][0])  # Obtain the width of the area in pixels

    np_area = np.array(new_area_coords)         # Convert the new coords to a numpy array for easier work
    max_height = np.max(np_area[:,1])           # Store highest v value of the entire area
    min_height = np.min(np_area[:,1])           # Store lowest v value of the entire area
    start_u = np.min(np_area[:,0])              # Save the smallest u value for the start of the area
    
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
        if not coverageCheck(config.uav.heading_angle,start_u,pass_shift,coverage_width,coverage_height):   # Check if coverage is still complete
            print("Coverage is no longer complete, adding another pass")
            # Check if another pass is required
            # If another pass is required, add another then recenter the passes
            number_of_passes += 1
            remainder = length_of_area - (number_of_passes * coverage_width - (number_of_passes-1)*config.side_overlap*coverage_width) 
            pass_shift = remainder/2
        else:
            print("Another pass is not required due to the current configuration allowing complete coverage")
    else:
        print("There is no wind and therefore no heading angle compensation")

    # Create polygon and NFZ paths to check for intersections with passes
    polygon_path = sg.Polygon(new_area_coords)
    NFZ_paths = []
    for NFZ in new_NFZs:
        NFZ_path = sg.Polygon(NFZ)
        NFZ_paths.append(NFZ_path)

    # Set maximum altitude difference
    max_alt_diff = max_uav_alt - min_uav_alt

    # Shift passes to allow for even distribution
    u = start_u + coverage_width/2 + pass_shift
    for i in range(0,number_of_passes):        # Cycle through all full-length passes across entirety of area
        # Find points where passes intersect with ROI

        intersection_points = []    # Points of intersection for each pass
        pass_edge = sg.LineString([(u,min_height-1),(u,max_height+1)]) # Create oversized pass to check for intersections

        # Fixed bug with edge crossing, tried removing duplicates
        intersection = polygon_path.intersection(pass_edge)
        if intersection.is_empty:   # If there is no intersection with the ROI, must be an error
            print("Pass did not intersect anywhere on the ROI")
            exit(1)
        elif type(intersection) == sg.Point:        # If the intersection is at a single point (Corner of the path), add the point twice
            intersection_points.append([intersection.x,intersection.y])
            intersection_points.append([intersection.x,intersection.y])
        elif type(intersection) == sg.LineString:   # If the intersection is multiple coordinates, cycle through and add all
            for point in intersection.coords:
                intersection_points.append([point[0],point[1]])

        # Find where passes intersect with NFZs
        for NFZ_path in NFZ_paths:
            intersection = NFZ_path.intersection(pass_edge)
            if intersection.is_empty:   # If there is no intersection with an NFZ, continue
                continue
            elif type(intersection) == sg.Point:        # If the intersection is at a single point (Corner of the path), add the point twice
                intersection_points.append([intersection.x,intersection.y])
                intersection_points.append([intersection.x,intersection.y])
            elif type(intersection) == sg.LineString:   # If the intersection is multiple coordinates, cycle through and add all
                for point in intersection.coords:
                    intersection_points.append([point[0],point[1]])

        points_on_pass = sorted(intersection_points,key=lambda point:point[1])   # List vertically
        subpasses = len(points_on_pass)/2   # Calculate the number of subpasses
        
        for j in range(0,int(subpasses)):   # Split full length passes into sub passes if obstacles are found
            start = points_on_pass[j*2]
            end = points_on_pass[j*2 + 1]
            pass_length = get_distance(start,end)-coverage_height    # Calculate the pass length and remove the height of the camera footprint
            if pass_length <= config.min_pass_length:               # Check to see if pass is to size
                print("Pass length is too small")                   # If pass is too small, continue
                continue
            elif pass_length > config.max_pass_length:
                print("Pass length is too large, aborting")
                exit(1)

            v = start[1] + coverage_height/2  # Shift pass up by half the size of the coverage height

            # Get altitude data
            altitude_profile = get_altitude_profile(pass_length,terrain,uav_altitude,u,v,wind_angle)
            # Create terraces from the pass
            image_passes = create_terraces(u,v,altitude_profile,wind_angle,pass_length,image_passes,max_alt_diff,config.min_pass_length)
        u += distance_between_photos_width          # Increase U value on each loop

    # Print stats
    print(f"length = {length_of_area}")
    print(f"Footprint of image: {coverage_width}x{coverage_height}")
    print(f"Distance between passes: {round(distance_between_photos_width,2)} m")
    print(f"number_of_passes = {number_of_passes}")
    print(f"Min altitude: {round(min_uav_alt,2)} m\nMax altitude: {round(max_uav_alt,2)}m"
            +f"\nDesired altitude: {round(uav_altitude,2)}")

    return image_passes