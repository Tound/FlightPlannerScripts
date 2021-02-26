# Modified TSP

#!/usr/bin/env python
"""
PROTOTYPE
Modified TSP for photogrammety shortest route finding
Based on TurboFart GitHub
"""

import math
import random
import matplotlib.pyplot as pyplot
import numpy as np
from dubins_3D import *
#from dubins import *
import shapely.geometry

g = 9.81

MAX_TAX = 1*10**12

class ImageLocation:
    def __init__(self,x,y,altitude):
        self.x = x
        self.y = y
        self.altitude = altitude
        self.angle = None

    def energyTo(self, other_img_loc, routemanager):
        d_path = None

        TAX = 1     # Apply tax to undesired routes
        # Check if heading angle is equal to 
        if not ((self.angle <= routemanager.pass_angle + math.radians(2) and self.angle >= routemanager.pass_angle - math.radians(2)) or 
        (self.angle+math.pi <= routemanager.pass_angle + math.radians(2) and self.angle+math.pi >= routemanager.pass_angle - math.radians(2))):
            #print("TAXED")
            if ((self.angle >= routemanager.wind_angle + math.radians(2)) or (self.angle <= routemanager.wind_angle - math.radians(2))):
                TAX = 10
            else:
                TAX = 100

        # SECURITY FUNCTION
        # Ensure not in NFZ
        line = shapely.geometry.LineString([[self.x,self.y],[other_img_loc.x,other_img_loc.y]])
        for NFZ in routemanager.NFZs:
            if line.intersects(NFZ):
                return MAX_TAX,d_path
        
        # Add spiral
        if self.angle != other_img_loc.angle:
            q0 = (self.x, self.y,self.altitude,self.angle)
            q1 = (other_img_loc.x,other_img_loc.y,other_img_loc.altitude,other_img_loc.angle)
            #q0 = (self.x, self.y,self.angle)
            #q1 = (other_img_loc.x,other_img_loc.y,other_img_loc.angle)
            #print(q0,q1,routemanager.min_turn)
            d_path = dubins_shortest_path(q0,q1,routemanager.min_turn)
            return TAX*d_path.length(),d_path
        else:
            dx = self.x - other_img_loc.x
            dy = self.y - other_img_loc.y
            dz = self.altitude - other_img_loc.altitude
            angle_between_points = math.tan(dz/math.sqrt(dx*dx +dy*dy))
            if abs(angle_between_points) > routemanager.max_grad:
                #spiral
                xEnergy = abs(dx)
                yEnergy = abs(dy)
                zEnergy = abs(dz)
            else:
                xEnergy = abs(dx)
                yEnergy = abs(dy)
                zEnergy = abs(dz)

            if (self.altitude - other_img_loc.altitude) < 0:
                altEnergy = routemanager.uav_mass*g* abs(self.altitude - other_img_loc.altitude)
            else:
                altEnergy = 0 # If the next point is below the current
                #return TAX*TAX,d_path
            return TAX*(math.sqrt(xEnergy*xEnergy + yEnergy*yEnergy + zEnergy*zEnergy) + altEnergy),d_path

    def __repr__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.altitude)

    def setAngle(self,angle):
        self.angle = angle

    def getAngle(self):
        return self.angle
    
    def setAngle(self,angle):
        self.angle = angle

class RouteManager:
    all_image_locations = []
    pass_angle = None
    wind_angle = None
    min_turn = None
    uav_mass = None
    NFZs = []
    max_grad = None
    def addImageLocation(self,image_location):
        self.all_image_locations.append(image_location)
    
    def getImageLocation(self,index):
        return self.all_image_locations[index]
    
    def numberOfLocations(self):
        return len(self.all_image_locations)

    def setParams(self,pass_angle,wind_angle,min_turn,uav_mass,NFZs,max_grad):
        self.pass_angle = pass_angle
        self.wind_angle = wind_angle
        self.min_turn = min_turn
        self.uav_mass = uav_mass
        self.NFZs = NFZs
        self.max_grad = max_grad


class Route:
    dubins_paths = []
    def __init__(self,routemanager,route=None):
        self.routemanager = routemanager
        self.route = []
        self.fitness = 0.0
        self.energy = 0
        if route is not None:
            self.route = route
        else:
            for i in range(0,self.routemanager.numberOfLocations()):
                self.route.append(None)

    def __len__(self):
        return len(self.route)

    def __getitem__(self,index):
        return self.route[index]

    def __setitem__(self,index,value):
        self.route[index] = value

    def __repr__(self):
        geneString = ""
        for i in range(0, self.routeSize()):
            geneString += "(" + str(self.getImageLocation(i)) + "),"
        return geneString[:len(geneString)-1]

    def getRoute(self):
        return self.route

    def generateIndividual(self):
        for index in range(0,self.routemanager.numberOfLocations()):
            self.setImageLocation(index,self.routemanager.getImageLocation(index))

    def getImageLocation(self,index):
        return self.route[index]

    def setImageLocation(self,index,image_location):
        self.route[index] = image_location
        self.fitness = 0.0
        self.energy = 0

    def getFitness(self):
        if self.fitness == 0:
            self.fitness = 1/float(self.getEnergy())
        return self.fitness
    
    def getEnergy(self):
        # COST FUNCTION
        self.dubins_paths = []
        if self.energy == 0:
            routeEnergy = 0
            # Reset heading angles for new ones to be calculated
            for location in self.route:
                location.setAngle(None)

            for index in range(0,self.routeSize()):

                prev_location = self.getImageLocation(index-1)
                current_location = self.getImageLocation(index)
                destination_location = None

                if index+1 < self.routeSize():
                    destination_location = self.getImageLocation(index+1)
                else:
                    destination_location = self.getImageLocation(0)
                    following_location = self.getImageLocation(1)

                if index+2 < self.routeSize():
                    following_location = self.getImageLocation(index+2)
                else:
                    following_location = self.getImageLocation(index+2 - self.routeSize())

                current_dx = current_location.x - prev_location.x
                current_dy = current_location.y - prev_location.y
                current_heading = math.atan2(current_dy,current_dx)

                if current_location.getAngle() is None: current_location.setAngle(current_heading)

                # Calculate the angle of the next heading
                next_dx = following_location.x - destination_location.x
                next_dy = following_location.y - destination_location.y
                try:
                    next_heading = math.atan(next_dy/next_dx)   # Could be 0
                except RuntimeError:
                    next_heading = math.inf

                # If the next location doesnt have a heading
                if destination_location.getAngle() is None: destination_location.setAngle(next_heading)

                #print(current_location.angle,destination_location.angle)

                energy,dpath = current_location.energyTo(destination_location,self.routemanager)
                if dpath is not None:
                    self.dubins_paths.append(dpath)
                routeEnergy += energy
            self.energy = routeEnergy
        return self.energy
    
    def getDPaths(self):
        return self.dubins_paths

    def routeSize(self):
        return len(self.route)

    def containsLocation(self, location):
        return location in self.route


class Population:
    def __init__(self,routemanager,populationSize,initialise):
        self.routes = []
        for i in range(0,populationSize):
            self.routes.append(None)

        if initialise:
            for i in range(0,populationSize):
                newRoute = Route(routemanager)
                newRoute.generateIndividual()
                self.saveRoute(i,newRoute)
    
    def saveRoute(self,index,route):
        self.routes[index] = route

    def getRoute(self, index):
        return self.routes[index]

    def getFittest(self):
        fittest = self.routes[0]
        #print(self.routes[0].getRoute())
        for i in range(0, self.populationSize()):
            if fittest.getFitness() <= self.getRoute(i).getFitness():
                fittest = self.getRoute(i)
        return fittest

    def populationSize(self):
        return len(self.routes)


class GA:
    def __init__(self,routemanager,mutationRate, tournamentSize):
        self.routemanager = routemanager
        self.mutationRate = mutationRate
        self.tournamentSize = tournamentSize
        self.elitism = True

    def evolvePopulation(self, population):
        newPopulation = Population(self.routemanager, population.populationSize(),False)
        elitismOffset = 0
        if self.elitism:
            newPopulation.saveRoute(0,population.getFittest())
            elitismOffset = 1

        for i in range(elitismOffset, newPopulation.populationSize()):
            parent1 = self.tournamentSelection(population)
            parent2 = self.tournamentSelection(population)
            child = self.crossover(parent1,parent2)
            newPopulation.saveRoute(i,child)

        for i in range(elitismOffset, newPopulation.populationSize()):
            self.mutate(newPopulation.getRoute(i))

        return newPopulation


    def crossover(self,parent1,parent2):
        child = Route(self.routemanager)

        startPos = int(random.random() * parent1.routeSize())
        endPos = int(random.random() * parent1.routeSize())

        for i in range(0,child.routeSize()):
            if startPos < endPos and i > startPos and i < endPos:
                child.setImageLocation(i,parent1.getImageLocation(i))
            elif startPos > endPos:
                if not (i< startPos and i > endPos):
                    child.setImageLocation(i,parent1.getImageLocation(i))
        
        for i in range(0,parent2.routeSize()):
            if not child.containsLocation(parent2.getImageLocation(i)):
                for j in range(0,child.routeSize()):
                    if child.getImageLocation(j) == None:
                        child.setImageLocation(j,parent2.getImageLocation(i))
                        break
        return child

    def mutate(self,route):
        for routePos1 in range(0,route.routeSize()):
            if random.random() < self.mutationRate:
                routePos2 = int(route.routeSize()*random.random())

                location1 = route.getImageLocation(routePos1)
                location2 = route.getImageLocation(routePos2)

                route.setImageLocation(routePos2,location1)
                route.setImageLocation(routePos1,location2)
                
    def tournamentSelection(self, population):
        tournament = Population(self.routemanager, self.tournamentSize, False)
        for i in range(0,self.tournamentSize):
            randomId = int(random.random() * population.populationSize())
            tournament.saveRoute(i,population.getRoute(randomId))
        fittest = tournament.getFittest()
        return fittest


if __name__ == '__main__':
    # TEST
    all_image_locations = [(0,50,10),(10,10,0),(50,70,40),(70,90,0),(90,80,0),(40,40,0),(60,70,0),(40,90,100),(80,80,0)]
    routemanager = RouteManager()
    i=0
    for location in all_image_locations:
        image_location = ImageLocation(location[0],location[1],location[2])
        routemanager.addImageLocation(image_location)
        #print(routemanager.getImageLocation(i))
        i+=1

    # Initialize population
    pop = Population(routemanager, routemanager.numberOfLocations()+500, True)
    print( "Initial distance: " + str(pop.getFittest().getEnergy()))

 
    # Evolve population for 50 generations
    ga = GA(routemanager,0.015,20)
    pop = ga.evolvePopulation(pop)
    for i in range(0, 10):
        pop = ga.evolvePopulation(pop)
        print(f"{i/10} %")


    print(pop.getFittest())
    route_x = np.array([])
    route_y = np.array([])
    route_z = np.array([])

    bestRoute = pop.getFittest()
    for location in bestRoute:
        route_x = np.append(route_x,location.x)
        route_y = np.append(route_y,location.y)
        route_z = np.append(route_z,location.altitude)

    plt.show()