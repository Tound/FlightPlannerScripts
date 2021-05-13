"""
These classes are to be used with the Efficient flight planning software
Created by Thomas Pound for the MEng Project
The classes include an Image_Pass and Image_Location
Last updated 30/4/21
"""

import math
from dubins_3D import *
import shapely.geometry as sg
from create_spirals import *

G = 9.81            # Acceleration caused by gravity in m/s^2
MAX_TAX = 1*10**12  # Maximum tax added to routes that have sections that are considered illegal or unwanted

class Image_Pass:
    """
    Image pass class to represent every image pass for the TSP
    Config is used to determine whether the pass is in the standard direction 
    or to be reversed.
    If config is True, the pass should be used as normal
    If config is False, the pass should be used in reverse
    """
    def __init__(self,start,end,altitude,wind_angle):
        self.wind_angle = wind_angle        # Wind angle in radians
        self.energy = [None,None]           # Energy for each configuration
        self.length = None                  # Length of the pass
        self.heading = [None,None]          # Heading angles for each configuration of the pass in radians
        self.start = start                  # Start location of the pass
        self.end = end                      # End location of the pass
        self.altitude = altitude            # Altitude of the pass

    def getStart(self,config):  # Obtain start location of a pass for a particular configuration
        if config:
            return self.start
        else:
            return self.end

    def getEnd(self,config):    # Get end location of the pass for a known configuration
        if config:
            return self.end
        else:
            return self.start

    def getLength(self):
        """
        Get length of pass for a known configuration
        """
        if self.length is None:
            length = 0

            dx = self.end[0] - self.start[0]
            dy = self.end[1] - self.start[1]
            dz = self.end[2] - self.start[2]
            length += math.sqrt(dx*dx + dy*dy + dz*dz)
            self.length = length
        return self.length

    def __repr__(self):
        string = ''
        string += f"({self.start},{self.end},{self.altitude}),"
        return string[:-1]

    def getHeading(self,config):
        """
        Retrieve or calculate the heading angle for any config
        Uses the direction of the pass to calculate the heading angle
        Parameters:
            config  - Boolean value to choose which direction the pass should be traversed
                      True = Normal direction
                      False = Reverse Direction
        Returns:
            The heading for the specified configuration
        """
        if config:
            if self.heading[0] is None:
                if self.start == self.end :
                    heading = self.wind_angle - math.pi/2
                    if heading >= 2*math.pi:
                        heading -= 2*math.pi
                    elif heading <= -2*math.pi:
                        heading += 2*math.pi

                    self.heading[0] = heading
                    return self.heading[0]

                else:
                    dx = self.end[0] - self.start[0]
                    dy = self.end[1] - self.start[1]
                    heading = math.atan2(dy,dx)

                    self.heading[0] = heading
            return self.heading[0]

        else:
            if self.heading[1] is None:
                if self.start == self.end:
                    self.heading[1] = self.wind_angle + math.pi/2
                    return self.heading[1]
                else:
                    dx = self.start[0] - self.end[0]
                    dy = self.start[1] - self.end[1]
                    heading = math.atan2(dy,dx)

                    self.heading[1] = heading
            return self.heading[1]

    def energyTo(self,config,other_pass,other_pass_config,routemanager):
        """
        Calculate the energy require to traverse between two passes
        Parameters:
            config              - Configuration of pass for direction of traversal
            other_pass          - The next pass that on the route
            other_pass_config   - The configuration of the next pass in the route, decides the direction of traversal
            routemanger         - Routemanager that holds all the flight settings

        Returns:
            energy              - The respective energy to travel to the next pass
            dpath               - The dubins path required to link the two passes
        """
        d_path = None    
        spiral = None
        energy = 0

        end = self.getEnd(config)
        start = other_pass.getStart(other_pass_config)
        q0 = (end[0],end[1],end[2],self.getHeading(config))
        q1 = (start[0],start[1],start[2],other_pass.getHeading(other_pass_config))

        dx = start[0] - end[0]
        dy = start[1] - end[1]
        dz = start[2] - end[2]
        if dz > 1:
            # Check if too steep
            incline_angle = math.atan2(dz, math.sqrt(dx*dx + dy*dy)) 
            if incline_angle > routemanager.max_grad:
                spiral = create_spiral(q0,dz,routemanager.min_turn,routemanager.max_grad)
                q0 = spiral.end
            alt_energy = routemanager.uav_mass*G* dz
        else:
            alt_energy = 1 # If the next point is below the current

        d_path = dubins_shortest_path(q0,q1,routemanager.min_turn)

        # Check if goes through NFZ
        # Create line
        edge = sg.LineString([(start[0],start[1]),(end[0],end[1])])
        for NFZ_edge in routemanager.NFZ_edges:
            intersection = NFZ_edge.getEdge().intersection(edge)
            if not intersection.is_empty:
                energy = MAX_TAX
                return energy,d_path,spiral

        if d_path.get_length() == 0:
            d_path = None
            energy = 0
        else:
            energy += d_path.get_length()

        if spiral != None:
            energy += spiral.get_length()

        energy = energy*alt_energy

        return energy,d_path,spiral
