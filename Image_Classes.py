# These classes are to be used with the Efficient flight planning software
# created by Thomas Pound for the MEng Project
# The classes include an Image_Pass and Image_Location

import math
from dubins_3D import *

G = 9.81    # Acceleration caused by gravity in m/s^2

class Image_Pass:
    """
    Config is used to determine whether the pass is in the standard direction 
    or to be reversed.
    If config is True, the pass should be used as normal
    If config is False, the pass should be used in reverse
    """
    def __init__(self,image_locs,wind_angle):
        self.wind_angle = wind_angle
        self.image_locs = image_locs
        self.energy = [None,None]
        self.length = None
        self.heading = [None,None]

    def getStart(self,config):  # Obtain start location of a pass for a particular configuration
        if config:
            return self.image_locs[0]
        else:
            return self.image_locs[len(self.image_locs)-1]

    def getEnd(self,config):
        if config:
            return self.image_locs[len(self.image_locs)-1]
        else:
            return self.image_locs[0]

    def getLength(self,config):
        if self.length is None:
            length = 0
            for i,image_loc in enumerate(self.image_locs,0):
                if i+1 < len(self.image_locs):
                    dx = self.image_locs[i+1].x - image_loc.x
                    dy = self.image_locs[i+1].y - image_loc.y
                    dz = self.image_locs[i+1].altitude - image_loc.altitude
                    length += math.sqrt(dx*dx + dy*dy + dz*dz)
                else:
                    break
            self.length = length
        return self.length

    def getEnergy(self,config,routemanager):
        if config:
            if self.energy[0] is None:
                energy = 0
                for i,image_loc in enumerate(self.image_locs):
                    dx = image_loc.x - self.image_locs[i-1].x
                    dy = image_loc.y - self.image_locs[i-1].y
                    dz = image_loc.altitude - self.image_locs[i-1].altitude

                    if dz>0:
                        if math.atan2(dz,math.sqrt(dx*dx +dy*dy)) > routemanager.max_grad:
                            print("PASS IS TOO STEEP!")
                        gpe = routemanager.uav_mass*G*dz
                        energy += math.sqrt(dx*dx +dy*dy + dz*dz) + gpe
                    else:
                        energy += math.sqrt(dx*dx +dy*dy)
                self.energy[0] = energy
            return self.energy[0]

        else:
            if self.energy[1] is None:
                energy = 0
                for i,image_loc in reversed(list(enumerate(self.image_locs))):
                    dx = image_loc.x - self.image_locs[i-1].x
                    dy = image_loc.y - self.image_locs[i-1].y
                    dz = image_loc.altitude - self.image_locs[i-1].altitude
                    if dz>0:
                        if math.atan2(dz,math.sqrt(dx*dx +dy*dy)) > routemanager.max_grad:
                            print("PASS IS TOO STEEP!")
                        gpe = routemanager.uav_mass*G*dz
                        energy += math.sqrt(dx*dx +dy*dy + dz*dz) + gpe
                    else:
                        energy += math.sqrt(dx*dx +dy*dy)
                self.energy[1] = energy
            return self.energy[1]

    def __repr__(self):
        string = ''
        for image_loc in self.image_locs:
            string += f"({image_loc.x},{image_loc.y},{image_loc.altitude}),"
        return string[:-1]

    def getHeading(self,config):
        if config:
            if self.heading[0] is None:
                if len(self.image_locs) <2:
                    heading = self.wind_angle - math.pi/2
                    if heading >= 2*math.pi:
                        heading -= 2*math.pi
                    elif heading <= -2*math.pi:
                        heading += 2*math.pi

                    self.heading[0] = heading
                    return self.heading[0]

                else:
                    dx = self.image_locs[1].x - self.image_locs[0].x
                    dy = self.image_locs[1].y - self.image_locs[0].y
                    heading = math.atan2(dy,dx)

                    self.heading[0] = heading
            return self.heading[0]

        else:
            if self.heading[1] is None:
                if len(self.image_locs) < 2:
                    self.heading[1] = self.wind_angle + math.pi/2
                    return self.heading[1]
                else:
                    dx = self.image_locs[len(self.image_locs)-2].x - self.image_locs[len(self.image_locs)-1].x
                    dy = self.image_locs[len(self.image_locs)-2].y - self.image_locs[len(self.image_locs)-1].y
                    heading = math.atan2(dy,dx)

                    self.heading[1] = heading
            return self.heading[1]

    def energyTo(self,config,other_pass,other_pass_config,routemanager):
        d_path = None        
        # Add spiral?
        end = self.getEnd(config)
        start = other_pass.getStart(other_pass_config)
        q0 = (end.x,end.y,end.altitude,self.getHeading(config))
        q1 = (start.x,start.y,start.altitude,other_pass.getHeading(other_pass_config))

        d_path = dubins_shortest_path(q0,q1,routemanager.min_turn)

        dz = end.altitude - start.altitude
        if dz > 0:
            # Check if too steep
            if math.atan2(dz, d_path.length_3d()) > routemanager.max_grad:
                print("DUBINS IS TOO STEEP!")
            altEnergy = routemanager.uav_mass*G* dz
        else:
            altEnergy = 0 # If the next point is below the current

        if d_path.length() == 0:
            d_path = None
            energy = 0
        else:
            energy = d_path.length() + altEnergy

        return energy,d_path

class Image_Location:
    def __init__(self,x,y,altitude):
        self.x = x
        self.y = y
        self.altitude = altitude
