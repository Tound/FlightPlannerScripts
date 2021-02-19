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
from dubinspython import *

uav_mass = 18
g = 9.81

class ImageLocation:
    heading_angle = math.pi
    def __init__(self,x,y,altitude):
        self.x = x
        self.y = y
        self.altitude = altitude

    def energyTo(self, other_img_loc):
        #angle_between
        xEnergy = abs(self.x - other_img_loc.x)
        yEnergy = abs(self.y - other_img_loc.y)
        if (self.altitude - other_img_loc.altitude) < 0:
            zEnergy = uav_mass*g* abs(self.altitude - other_img_loc.altitude)
        else:
            zEnergy = 0
        return math.sqrt(xEnergy*xEnergy + yEnergy*yEnergy) + zEnergy

    def __repr__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.altitude)

class RouteManager:
    all_image_locations = []
    def addImageLocation(self,image_location):
        self.all_image_locations.append(image_location)
    
    def getImageLocation(self,index):
        return self.all_image_locations[index]
    
    def numberOfLocations(self):
        return len(self.all_image_locations)


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
        if self.energy == 0:
            routeEnergy = 0
            for index in range(0,self.routeSize()):
                from_location = self.getImageLocation(index)
                destination_location = None
                #prev_location = self.getImageLocation(index-1)
                if index+1 < self.routeSize():
                    destination_location = self.getImageLocation(index+1)
                else:
                    destination_location = self.getImageLocation(0)

                routeEnergy += from_location.energyTo(destination_location)
            self.energy = routeEnergy
        return self.energy
    
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

    #plt.show()