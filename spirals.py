# Spirals
import matplotlib.pyplot as plt
import numpy as np
import math

class section:
    def __init__(self):
        self.type = 
        self.start = 
        self.end = 
        
class spiral:
    def __init__(self):
        self.sections = []
        self.radius = 0
    def length(self):
        return section[0].length + section[1].length

def sample_helix(timeStep,helix):
    tprime = t/helix.radius
    total_length = helix.length()
    t1 = helix.segments[0]
    if len(helix.segments) > 1
        t2 = helix.segments[1]
    else:
        t2 = None
    while t < total_length:


        
        t += tprime
# TO AVOID OBSTACLES
# IF SPIRAL UP, DO SPIRAL FIRST
# IF SPRIAL DOWN, DO STRAIGHT FIRST

def createHelix(circum,height,max_incline_grad):
    turn_height = circum*max_incline_grad
    number_of_turns = height/turn_height
    distance = math.sqrt(height*height + number_of_turns*circum)
    return Helix()

def createSpiralRoute(current_loc,destination_loc,max_incline_grad,turn_rad):
    circum = math.pi*2*turn_rad
    segments = []
    dx = destination_loc[0] - current_loc[0]
    dy = destination_loc[1] - current_loc[1]
    dz = destination_loc[2] - current_loc[2]

    if dz>0:
        #reachable_alt = math.sqrt(dx*dx + dy*dy) * math.atan(max_incline_angle)
        reachable_alt = math.sqrt(dx*dx + dy*dy) * max_incline_grad
        if reachable_alt >= dz:
            return True
        remaining_alt = dz - reachable_alt
        distance = math.sqrt(dx*dx + dy*dy + reachable_alt*reachable_alt)
        segments.append(distance,'straight')
        segments.append(createHelix(circum,remaining_alt,max_incline_grad))
    else:
        reachable_alt = math.sqrt(dx*dx + dy*dy) * -max_incline_grad
        remaining_alt = dz - reachable_alt
        distance = math.sqrt(dx*dx + dy*dy + reachable_alt*reachable_alt)
        segments.append(distance,'straight')
        segments.append(createHelix(circum,remaining_alt,max_incline_grad))




# UAV SETTINGS
max_incline_angle = 30
max_incline_grad = math.tan(max_incline_angle)
current_loc = (20,20,20)
destination_loc = (20,100,100)

createSpiralRoute(current_loc,destination_loc,max_incline_grad)