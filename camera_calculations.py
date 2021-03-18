# Camera Calculations
def getAltitude(focal_length,coverage_x,sensor_x):
    altitude = coverage_x*focal_length/sensor_x
    return altitude

def imageDimensions(resolution, aspect_ratio):
    aspect_ratio = aspect_ratio.split(':')
    width_ratio = aspect_ratio[0]
    height_ratio = aspect_ratio[1]
    imageWidth = 0
    imageHeight = 0
    return imageWidth, imageHeight

def getCoverageResolution(altitude):
    coverage_resolution = 0
    return coverage_resolution

