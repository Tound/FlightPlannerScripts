# Camera Calculations

import math

def getAltitude(focal_length,coverage_x,sensor_x):
    altitude = coverage_x*focal_length/sensor_x
    return altitude

def imageDimensions(resolution, aspect_ratio):
    width_ratio = aspect_ratio[0]
    height_ratio = aspect_ratio[1]
    ratio = width_ratio/height_ratio
    imageWidth = math.sqrt(resolution*10**6 * ratio)
    imageHeight = imageWidth/ratio
    return imageWidth, imageHeight

def getCoverageResolution(altitude):
    coverage_resolution = 0
    return coverage_resolution

def getCoverageSize(coverage_resolution,image_x,image_y):
    coverage_width = coverage_resolution*image_x
    coverage_height = coverage_resolution*image_y
    return coverage_width, coverage_height

