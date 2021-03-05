# Camera Calculations
def getAltitude(focal_length,coverage_x,sensor_x):
    altitude = coverage_x*focal_length/sensor_x
    return altitude