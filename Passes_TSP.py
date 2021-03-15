# Modified TSP

#!/usr/bin/env python
"""
PROTOTYPE
Modified TSP for photogrammety shortest route finding
Based on TurboFart GitHub
"""

import math
import random
import matplotlib.pyplot as plt
import numpy as np
from dubins_3D import *
from create_passes import *
from Image_Classes import *
import shapely.geometry

import time

MAX_TAX = 1*10**12

class RouteManager:
    all_image_passes = []
    wind_angle = None
    min_turn = None
    uav_mass = None
    NFZs = []
    max_grad = None
    start_loc = None
    def addImagePass(self,image_pass):
        self.all_image_passes.append(image_pass)
    
    def getImagePass(self,index):
        return self.all_image_passes[index]
    
    def numberOfPasses(self):
        return len(self.all_image_passes)

    def setParams(self,wind_angle,min_turn,uav_mass,NFZs,max_grad,start_loc):
        self.wind_angle = wind_angle
        self.min_turn = min_turn
        self.uav_mass = uav_mass
        self.NFZs = NFZs
        self.max_grad = max_grad
        self.start_loc = start_loc


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
            for i in range(0,self.routemanager.numberOfPasses()):
                self.route.append([None,None])

    def __len__(self):
        return len(self.route)

    def __getitem__(self,index):
        return self.route[index]

    def __setitem__(self,index,value):
        self.route[index] = value

    def __repr__(self):
        geneString = ""
        for i in range(0, self.routeSize()):
            image_pass,pass_config = self.getImagePass(i)
            geneString += "(" + str(image_pass) + str(pass_config) + "),"
        return geneString[:len(geneString)-1]

    def getRoute(self):
        return self.route

    def generateIndividual(self):
        for index in range(0,self.routemanager.numberOfPasses()):
            image_pass = self.routemanager.getImagePass(index)

            self.setImagePass(index,image_pass,random.choice([True,False]))

    def getImagePass(self,index):
        return self.route[index][0],self.route[index][1]

    def setImagePass(self,index,image_pass,config):
        self.route[index] = [image_pass,config]
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
            route_energy = 0
            # Energy from start to first
            route_energy += 10
            for index in range(0,self.routeSize()):
                current_pass,current_pass_config = self.getImagePass(index)
                destination_pass = None
                route_energy += current_pass.getEnergy(current_pass_config,self.routemanager.uav_mass)   # Add energy to traverse path

                if index+1 < self.routeSize():
                    destination_pass,destination_pass_config = self.getImagePass(index+1)
                else:
                    destination_pass,destination_pass_config = self.getImagePass(0)

                energy,dpath = current_pass.energyTo(current_pass_config,destination_pass,destination_pass_config,self.routemanager)
                if dpath is not None:
                    self.dubins_paths.append(dpath)
                route_energy += energy
            route_energy += 10           # Energy from end point to start pos
            self.energy = route_energy
        return self.energy
    
    def getDPaths(self):
        return self.dubins_paths

    def routeSize(self):
        return len(self.route)

    def containsPass(self, image_pass):
        routeNp = np.array(self.route)
        if len(np.where(routeNp[:,0] == image_pass)[0]) > 0:
            return True
        else:
            return False 

    # Added by Thomas Pound
    def orderPasses(self,index):
        """
        This function orders the passes so that the start location is first
        """
        self.route = np.roll(self.route,-index)


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
            p1_pass,p1_config = parent1.getImagePass(i)
            if startPos < endPos and i > startPos and i < endPos:
                child.setImagePass(i,p1_pass,p1_config)
            elif startPos > endPos:
                if not (i< startPos and i > endPos):
                    child.setImagePass(i,p1_pass,p1_config)
        
        for i in range(0,parent2.routeSize()):
            p2_pass,p2_config = parent2.getImagePass(i)
            if not child.containsPass(p2_pass):
                for j in range(0,child.routeSize()):
                    c_pass,c_config = child.getImagePass(j)
                    if c_pass == None:
                        child.setImagePass(j,p2_pass,p2_config)
                        break

        return child

    def mutate(self,route):
        for routePos1 in range(0,route.routeSize()):
            if random.random() < self.mutationRate:
                routePos2 = int(route.routeSize()*random.random())

                image_pass1,p1_config = route.getImagePass(routePos1)
                image_pass2,p2_config = route.getImagePass(routePos2)

                if image_pass1 is None or image_pass2 is None:
                    exit(1)

                route.setImagePass(routePos2,image_pass1,random.choice([True,False]))
                route.setImagePass(routePos1,image_pass2,random.choice([True,False]))
                
    def tournamentSelection(self, population):
        tournament = Population(self.routemanager, self.tournamentSize, False)
        for i in range(0,self.tournamentSize):
            randomId = int(random.random() * population.populationSize())
            tournament.saveRoute(i,population.getRoute(randomId))
        fittest = tournament.getFittest()
        return fittest

def TSP(image_passes,wind_angle,min_turn,uav_mass,NFZs,max_grad,start_loc,population_size=50,mutationRate=0.015,generations=50,tournamentSize=20):
    """
    Travelling salesman problem
    """

    # Create a pass for the start location
    start_point = Image_Location(start_loc[0],start_loc[1],start_loc[2])
    start_pass = Image_Pass([start_point],wind_angle)
    image_passes.append(start_pass)
    routemanager = RouteManager()
    routemanager.setParams(wind_angle,min_turn,uav_mass,NFZs,max_grad,start_loc)
    for image_pass in image_passes:
        routemanager.addImagePass(image_pass)

    pop = Population(routemanager,population_size,True)

    # Evolve population for the specified number of generations
    ga = GA(routemanager,mutationRate,tournamentSize)
    pop = ga.evolvePopulation(pop)
    for i in range(0, generations):
        pop = ga.evolvePopulation(pop)
        print(f"{100 * i/generations} %")

    bestRoute = pop.getFittest()    # Get best route

    index = np.where(np.array(bestRoute)[:,0] == start_pass)[0][0]
    bestRoute.orderPasses(index)

    return bestRoute


if __name__ == '__main__':
    # TEST
    image_locs1 = [Image_Location(10,10,120),Image_Location(20,20,120),Image_Location(30,30,120),Image_Location(40,40,120)]
    image_locs2 = [Image_Location(100,100,100),Image_Location(200,200,100),Image_Location(300,300,100),Image_Location(400,400,100)]
    image_locs3 = [Image_Location(150,250,90),Image_Location(170,270,90),Image_Location(190,290,90),Image_Location(210,310,90)]
    image_locs4 = [Image_Location(205,205,10),Image_Location(210,205,10),Image_Location(220,205,10),Image_Location(230,205,10)]

    wind_angle = math.radians(-45)
    image_passes = [Image_Pass(image_locs1,wind_angle),
                    Image_Pass(image_locs2,wind_angle),
                    Image_Pass(image_locs3,wind_angle),
                    Image_Pass(image_locs4,wind_angle)]

    shortest_path = TSP(image_passes,wind_angle,20,20,None,20,(20,20,20),population_size=100,generations=100)
    print(shortest_path)
    dpaths = shortest_path.getDPaths()

    stepSize = 1

    dubinsX = np.array([])
    dubinsY = np.array([])
    dubinsZ = np.array([])
    for dpath in dpaths:
        points = dubins_path_sample_many(dpath,stepSize)
        for point in points:
            dubinsX = np.append(dubinsX,point[0])
            dubinsY = np.append(dubinsY,point[1])
            dubinsZ = np.append(dubinsZ,point[2])


    fig = plt.figure(num=1,clear=True,figsize=(12,8))
    ax = fig.add_subplot(1,1,1,projection='3d')
    ax.set_aspect(aspect='auto')
    fig.tight_layout()

    plt.plot(dubinsX,dubinsY,dubinsZ,'yo',zorder=15,markersize = 1)

    plt.show()