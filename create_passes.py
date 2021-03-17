# Create Passes
import math
import numpy as np
import shapely.geometry as sg
from dubins_3D import *
from scipy.interpolate import griddata
from Image_Classes import *

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

    distance_between_photos_width = coverage_width - coverage_width*config.side_overlap
    distance_between_photos_height = coverage_height - coverage_height*config.forward_overlap

    # Obtain properties about the area
    sorted_vertices = sorted(new_area_coords, key=lambda u:u[0])
    length_of_area = sorted_vertices[len(sorted_vertices)-1][0] - sorted_vertices[0][0]
    print(f"{convertCoords([sorted_vertices[len(sorted_vertices)-1]],wind_angle,'xy')},{convertCoords([sorted_vertices[0]],wind_angle,'xy')}")

    np_area = np.array(new_area_coords)
    max_height = np.max(np_area[:,1])           # Store highest V value
    min_height = np.min(np_area[:,1])           # Store lowest V value
    start_u = np.min(np_area[:,0])

    # Calculate number of passes to cover area
    #number_of_passes = (length_of_area-coverage_width/2)/distance_between_photos_width
    
    number_of_passes = (length_of_area-config.side_overlap*coverage_width)/distance_between_photos_width
    # Overlap must be above 0 or the pass shift will not work
    if (number_of_passes % 1 > 0 or config.wind[0] > 0) and config.side_overlap > 0: # If the number of passes is not an integer or wind is present
        number_of_passes += 1
    if config.side_overlap == 0:
        print("SIDE OVERLAP IS 0 AND WILL CAUSE AN ISSUE!")

    # Create shift value to center the passes
    #remainder = length_of_area - coverage_width/2 - int(number_of_passes)*distance_between_photos_width
    remainder = length_of_area - (int(number_of_passes) * coverage_width - int(number_of_passes-1)*config.side_overlap*coverage_width) 
    #pass_shift = (coverage_width/2 + remainder)/2 - coverage_width/2
    pass_shift = remainder/2

    polygon_edges = []
    NFZ_edges = []

    # Create all edges
    for i in range(0,len(new_area_coords)):
        polygon_edges.append(Edge(new_area_coords[i-1][0],new_area_coords[i-1][1],
                                    new_area_coords[i][0],new_area_coords[i][1]))
    
    for NFZ_coords in new_NFZs:
        for i in range(0,len(NFZ_coords)):
            NFZ_edges.append(Edge(NFZ_coords[i-1][0],NFZ_coords[i-1][1],
                                    NFZ_coords[i][0],NFZ_coords[i][1]))


    # Shift passes to allow for even distribution
    u = start_u + coverage_width/2 + pass_shift
    print(f"U: {u}, Start u:{start_u},pass shift:{pass_shift}")

    #number_of_passes += 1

    print(distance_between_photos_width,coverage_width,number_of_passes,length_of_area)

    for i in range(0,int(number_of_passes)):        # Cycle through all full-length passes across entirety of area

        subpasses = 1

        pass_edge = sg.LineString([(u,min_height-1),(u,max_height+1)])
        max_intersect = (-math.inf,-math.inf)
        min_intersect = (math.inf,math.inf)
        for edge in polygon_edges:
            intersection = edge.getEdge().intersection(pass_edge)
            if intersection.is_empty:
                continue
            if intersection.y >= max_intersect[1]:               # Even though we are using uv axis, shapely requires xy
                max_intersect = [intersection.x,intersection.y]
            if intersection.y <= min_intersect[1]:
                min_intersect = [intersection.x,intersection.y]
        pass_start = min_intersect
        pass_end = max_intersect
        total_pass_length = getDistance(min_intersect,max_intersect)

        # Update pass edge?
        # Check if pass crosses any NFZ edges
        
        # FIX TO ALLOW FOR MULTIPLE NFZs
        intersection_points = []
        for edge in NFZ_edges:        # Cycle through all NFZs
            intersection = edge.getEdge().intersection(pass_edge)
            if intersection.is_empty:
                continue
            else:
                intersection_points.append([intersection.x,intersection.y])

        points_on_pass = [min_intersect,max_intersect]

        if not len(intersection_points)==0:
            for point in intersection_points:
                points_on_pass.append(point)

        points_on_pass = sorted(points_on_pass,key=lambda point:point[1])   # List vertically
        subpasses = len(points_on_pass)/2

        # Should center the images so there are no large gaps
        for j in range(0,int(subpasses)):
            image_locations = []
            start = points_on_pass[j*2]
            end = points_on_pass[j*2 + 1]
            pass_length = getDistance(start,end)

            #number_of_image_locations = (pass_length-coverage_height/2)/distance_between_photos_height

            number_of_image_locations = (pass_length-config.forward_overlap*coverage_height)/distance_between_photos_height

            if number_of_image_locations == 0:
                continue

            # Overlap must be above 0 or the pass shift will not work
            if number_of_image_locations % 1 > 0 and config.forward_overlap > 0: # If the number of image locations is not an integer or wind is present
                number_of_image_locations += 1
            if config.forward_overlap == 0:
                print("FORWARD OVERLAP IS 0 AND WILL CAUSE AN ISSUE!")
            # Need to calculate overhang of image footprint

            if config.forward_overlap < 0.5:
                # Add extra image location to account for incomplete coverage on corners
                number_of_image_locations += 1
            
            # Create vertical shift to make image locations have even distribution
            remainder = pass_length - (int(number_of_image_locations) * coverage_height - int(number_of_image_locations-1)*config.forward_overlap*coverage_height) 
            vertical_shift = remainder/2
            #remainder = pass_length - coverage_height/2 - number_of_image_locations*distance_between_photos_height
            #vertical_shift = (coverage_height/2 + remainder)/2 - coverage_height/2

            #number_of_image_locations +=1

            v = start[1] + coverage_height/2 + vertical_shift # Apply shift to center all image locations
            for j in range(0,int(number_of_image_locations)):

                coord = convertCoords([[u,v]],wind_angle,'xy')
                x = coord[0][0]
                y = coord[0][1]
                x_points = [int(x),int(x),int(x)+1,int(x)+1]
                y_points = [int(y),int(y)+1,int(y)+1,int(y)]
                z_points = [terrain[int(y)][int(x)],terrain[int(y)+1][int(x)],
                            terrain[int(y)+1][int(x)+1],terrain[int(y)][int(x)+1]]

                # For created terrain ONLY
                z = griddata((x_points,y_points),z_points,(x,y))    # Interpolate

                # Could average out the altiutude here>
                # Could run terraces?
                
                altitude = z + uav_altitude
                image_locations.append(Image_Location(x,y,altitude))

                v += distance_between_photos_height

            if len(image_locations) > 0:
                start_xy = convertCoords([start],wind_angle,'xy')
                end_xy = convertCoords([end],wind_angle,'xy')
                image_passes.append(Image_Pass(image_locations,config.wind[1]))
        u += distance_between_photos_width          # Increase U value on each loop

    print(f"length = {length_of_area}")
    print(coverage_width,coverage_height)
    print(distance_between_photos_width,distance_between_photos_height)
    print(f"number_of_passes = {number_of_passes}")

    return image_passes

