# Camera Calculations

import math

def get_altitude(focal_length,coverage_x,sensor_x):
    """
    Obtain the altitude from the camera settings
    """
    altitude = coverage_x*focal_length/sensor_x
    return altitude

def get_image_dimensions(resolution, aspect_ratio):
    """
    Obtain the size of the camera footprint in pixels
    """
    width_ratio = aspect_ratio[0]
    height_ratio = aspect_ratio[1]
    ratio = width_ratio/height_ratio
    image_width = math.sqrt(resolution*10**6 * ratio)
    image_height = image_width/ratio
    return image_width, image_height

def get_gsd(altitude, camera):
    coverage_resolution = 0
    return coverage_resolution

def get_coverage_size(coverage_resolution,image_x,image_y):
    coverage_width = coverage_resolution*image_x
    coverage_height = coverage_resolution*image_y
    return coverage_width, coverage_height

