# Create Passes
import math
import numpy as np
import shapely.geometry as sg
import matplotlib.pyplot as plt

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
        new_NFZ_coords = convertCoords(NFZ,wind_angle,'uv)
        new_NFZs.append(new_NFZ_coords)

    # Find footprint size
    coverage_width = config.coverage_resolution * camera.image_x
    coverage_height = config.coverage_resolution * camera.image_y

    uav_altitude = coverage_width *camera.focal_length/camera.sensor_x

    distance_between_photos_width = coverage_width - coverage_width*config.side_overlap
    distance_between_photos_height = coverage_height - coverage_height*config.forward_overlap

    pass_angle = math.pi/2  # In new coord system

    sorted_vertices = sorted(new_area_coords, key=lambda u:u[0])
    length_of_area = sorted_vertices[len(sorted_vertices)-1][0] - sorted_vertices[0][0]

    number_of_passes = (length_of_area-coverage_width)/distance_between_photos_width + 1

    polygon_edges = []
    NFZ_edges = []
    for i in range(0,number_of_passes):
        u += distance_between_photos_width
        image_locations = []

        

            pass_length = 
            number_of_image_locations = (pass_length-coverage_heights)/distance_between_photos_height

            # Need to calculate overhang of image footprint

            if config.forward_overlap < 0.5:
                # Add extra image location to account for incomplete coverage on corners
                number_of_image_locations += 1

            for j in range(0,number_of_image_locations):
                v += distance_between_photos_height
                x,y = convertCoords(u,v,'xy')
                altitude = terrain[x][y] + uav_altitude
                image_locations.append(Image_Location(x,y,altitude))

            # Center the image locations

            image_passes.append(Image_Pass(image_locations))

    print(f"length = {length_of_area}")
    print(coverage_width,coverage_height)
    print(distance_between_photos_width,distance_between_photos_height)
    print(f"number_of_passes = {number_of_passes}")

    return image_passes

