"""
Camera class and calculations
Calculations used to calculate various factors about the flight properties
Created by Thomas Pound
Last updated 19/5/21
"""
import math

class Camera:
    """
    Camera class holds all camera settings for a specific flight
    """
    def __init__(self,sensor_x,sensor_y,focal_length,resolution,aspect_ratio):
        self.sensor_x = sensor_x            # Width of camera sensor in m
        self.sensor_y = sensor_y            # Height of camera sensor in m
        self.focal_length = focal_length    # Focal length of the camera in m
        self.resolution = resolution        # Resolution of the camera in megapixels
        self.aspect_ratio = aspect_ratio    # Aspect ratio of the camera as a tuple in the form (x,y)

        # Get image dimensions in pixels
        self.image_x, self.image_y = self.get_image_dimensions()

    def get_altitude(self,coverage_width):
        """
        Obtain the altitude from the camera settings
        """
        altitude = coverage_width *self.focal_length/self.sensor_x
        return altitude

    def get_altitude_from_gsd(self,gsd):
        """
        Get the altitude from the ground sample distance
        """
        altitude = (self.image_x * gsd) * self.focal_length/self.sensor_x
        return altitude

    def get_image_dimensions(self):
        """
        Obtain the size of the camera image in pixels
        """
        # Split the aspect ratio into individual ratios
        width_ratio = self.aspect_ratio[0]
        height_ratio = self.aspect_ratio[1]
        ratio = width_ratio/height_ratio
        # Calculate the image dimensions
        image_width = math.sqrt(self.resolution*10**6 * ratio)
        image_height = image_width/ratio
        return image_width, image_height

    def get_gsd_from_alt(self,altitude):
        """
        Obtain the ground sample distance from the altitude
        """
        gsd = altitude*self.sensor_x/(self.focal_length * self.image_x)
        return gsd

    def get_coverage_size_gsd(self,gsd):
        """
        Get the coverage size of the camera footprint from the ground sample distance
        """
        coverage_width = gsd*self.image_x
        coverage_height = gsd*self.image_y
        return coverage_width, coverage_height

    def get_coverage_size_alt(self,altitude):
        """
        Get the coverage size of the camera footprint from the altitude
        """
        coverage_width = self.sensor_x*altitude/self.focal_length
        coverage_height = self.sensor_y*altitude/self.focal_length
        return coverage_width, coverage_height

