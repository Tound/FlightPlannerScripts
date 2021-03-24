# Create Passes
import math
import numpy as np
import shapely.geometry as sg
from dubins_3D import *
from scipy.interpolate import griddata
from Image_Classes_V2 import *

import time

G = 9.81

class Edge:
    def __init__(self,x1,y1,x2,y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.edge = sg.LineString([(x1,y1),(x2,y2)])

    def getEdge(self):
        return self.edge

def getAltitudeProfile(pass_length,terrain,uav_altitude,u,start,wind_angle):
    altitude_profile = []
    v = start[1]
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

def createPasses(area,NFZs,terrain,config):
    camera = config.camera
    wind_angle = config.wind[1]

    image_passes = []

    # IF WINDLESS
    if config.wind[0] == 0:
        # Find the largest edge
        wind_angle = 0 #largest_edge


    # Update coordinate system
    new_area_coords = convertCoords(area,wind_angle,'uv')
    new_NFZs = []
    for NFZ in NFZs:
        new_NFZ_coords = convertCoords(NFZ,wind_angle,'uv')
        new_NFZs.append(new_NFZ_coords)

    # Find footprint size
    coverage_width = config.coverage_resolution * camera.image_x
    coverage_height = config.coverage_resolution * camera.image_y

    uav_altitude = coverage_width *camera.focal_length/camera.sensor_x
    max_uav_alt = (camera.image_x * (config.coverage_resolution + 0.0015)) * camera.focal_length/camera.sensor_x
    min_uav_alt = (camera.image_x * (config.coverage_resolution - 0.0015)) * camera.focal_length/camera.sensor_x

    distance_between_photos_width = coverage_width - coverage_width*config.side_overlap

    # Obtain properties about the area
    sorted_vertices = sorted(new_area_coords, key=lambda u:u[0])
    length_of_area = sorted_vertices[len(sorted_vertices)-1][0] - sorted_vertices[0][0]
    print(f"{convertCoords([sorted_vertices[len(sorted_vertices)-1]],wind_angle,'xy')},{convertCoords([sorted_vertices[0]],wind_angle,'xy')}")

    np_area = np.array(new_area_coords)
    max_height = np.max(np_area[:,1])           # Store highest V value
    min_height = np.min(np_area[:,1])           # Store lowest V value
    start_u = np.min(np_area[:,0])
    
    number_of_passes = (length_of_area-config.side_overlap*coverage_width)/distance_between_photos_width
    # Overlap must be above 0 or the pass shift will not work
    if number_of_passes % 1 > 0 or config.wind[0] > 0: # If number of passes is not an integer to create complete coverage
        number_of_passes +=1 # If the number of passes is an integer and wind is present and the side overlap is above
        # Create shift value to center the passes
        remainder = length_of_area - (int(number_of_passes) * coverage_width - int(number_of_passes-1)*config.side_overlap*coverage_width) 
        pass_shift = remainder/2
    else:
        print("Shift is not required due to no wind and exact amount of passes")
        pass_shift = 0

    polygon_edges = []
    NFZ_edges = []

    # Create all polygon edges
    for i in range(0,len(new_area_coords)):
        polygon_edges.append(Edge(new_area_coords[i-1][0],new_area_coords[i-1][1],
                                    new_area_coords[i][0],new_area_coords[i][1]))
    
    # Create all NFZ edges
    for NFZ_coords in new_NFZs:
        for i in range(0,len(NFZ_coords)):
            NFZ_edges.append(Edge(NFZ_coords[i-1][0],NFZ_coords[i-1][1],
                                    NFZ_coords[i][0],NFZ_coords[i][1]))

    # Set terrace properties
    # Min terrace length
    min_length = 5
    max_alt_diff = max_uav_alt - min_uav_alt


    # Shift passes to allow for even distribution
    u = start_u + coverage_width/2 + pass_shift
    for i in range(0,int(number_of_passes)):        # Cycle through all full-length passes across entirety of area

        intersection_points = []
        pass_edge = sg.LineString([(u,min_height-1),(u,max_height+1)])
        max_intersect = (-math.inf,-math.inf)
        min_intersect = (math.inf,math.inf)
        for edge in polygon_edges:
            intersection = edge.getEdge().intersection(pass_edge)
            if intersection.is_empty:
                continue
            if type(intersection) == sg.Point:
                if intersection.y >= max_intersect[1]:               # Even though we are using uv axis, shapely requires xy
                    max_intersect = [intersection.x,intersection.y]
                if intersection.y <= min_intersect[1]:
                    min_intersect = [intersection.x,intersection.y]
                intersection_points.append([intersection.x,intersection.y])
            elif type(intersection) == sg.LineString:
                for point in intersection.coords:
                    if point[1] >= max_intersect[1]:
                        max_intersect = [point[0],point[1]]
                    if point[1] <= min_intersect[1]:
                        min_intersect = [point[0],point[1]]
                intersection_points.append([intersection.x,intersection.y])

        total_pass_length = getDistance(min_intersect,max_intersect)

        for edge in NFZ_edges:        # Cycle through all NFZs
            intersection = edge.getEdge().intersection(pass_edge)
            if intersection.is_empty:
                continue
            else:
                intersection_points.append([intersection.x,intersection.y])

        points_on_pass = sorted(intersection_points,key=lambda point:point[1])   # List vertically
        subpasses = len(points_on_pass)/2

        for j in range(0,int(subpasses)):   # Split full length passes into sub passes if obstacles are found
            start = points_on_pass[j*2]
            end = points_on_pass[j*2 + 1]
            pass_length = getDistance(start,end)

            # Get altitude data
            altitude_profile = getAltitudeProfile(pass_length,terrain,uav_altitude,u,start,wind_angle)

            v = start[1]
            current_terrace = []

            average_alt = 0
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
                                for alt in range(0,3):
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
                                # Create new terrace
                                terrace_start = current_terrace[0]
                                terrace_end = current_terrace[len(current_terrace)-1]
                                image_passes.append(Image_Pass(terrace_start,terrace_end,current_altitude,wind_angle))
                                current_terrace = []
                                current_altitude = 0

                        index += 1
            else:
                pass


            # Must make sure TSP does not go through land or through NFZ
        u += distance_between_photos_width          # Increase U value on each loop

    print(f"length = {length_of_area}")
    print(coverage_width,coverage_height)
    print(distance_between_photos_width)
    print(f"number_of_passes = {number_of_passes}")
    print(min_uav_alt,max_uav_alt,uav_altitude)

    return image_passes

