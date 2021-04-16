# Modified TSP

#!/usr/bin/env python
"""
Modified TSP for photogrammety shortest route finding
Based on TurboFart GitHub https://gist.github.com/turbofart/3428880#file-tsp-py
"""

import math
import random
import matplotlib.pyplot as plt
import numpy as np
from dubins_3D import *
from create_passes_V3 import *
from Image_Classes_V3 import *
import shapely.geometry

import time

MAX_TAX = 1*10**12  # Maximum tax added to routes that have sections that are considered illegal or unwanted

class RouteManager:
    """
    The routemanager class keeps track of all image passes and the other flight settings
    """
    all_image_passes = []
    wind_angle = None
    min_turn = None
    uav_mass = None
    NFZs = []
    NFZ_edges = []
    max_grad = None
    glide_slope = None
    start_loc = None
    def addImagePass(self,image_pass):
        self.all_image_passes.append(image_pass)

    def addImagePasses(self,image_passes):
        self.all_image_passes = image_passes
    
    def getImagePass(self,index):
        return self.all_image_passes[index]
    
    def numberOfPasses(self):
        return len(self.all_image_passes)

    def setParams(self,wind_angle,min_turn,uav_mass,NFZs,NFZ_edges,max_grad,glide_slope,start_loc):
        """
        Added by Thomas Pound so that flight parameters can be stored 
        and passed into functions
        """
        self.wind_angle = wind_angle
        self.min_turn = min_turn
        self.uav_mass = uav_mass
        self.NFZs = NFZs
        self.NFZ_edges = NFZ_edges
        self.max_grad = max_grad
        self.glide_slope = glide_slope
        self.start_loc = start_loc


class Route:
    """
    Route class represents an entire route that links all image passes
    """
    dubins_paths = []   # Stores all dubins path objects
    def __init__(self,routemanager,route=None):
        self.routemanager = routemanager
        self.route = []
        self.fitness = 0.0
        self.energy = 0
        self.length = 0
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
        """
        Function returns the "energy" of the route
        The energy is calculated using a cost function which takes into consideration the 
        elevation change and length of each pass
        """
        if self.energy == 0:    # If the energy of the route has not yet been calculated
            self.dubins_paths = []
            route_energy = 0    # Initialise energy as 0
            for index in range(0,self.routeSize()): # Cycle through every pass on route
                current_pass,current_pass_config = self.getImagePass(index) # Store the current pass and its configuration (forwards or backwards)
                destination_pass = None
                route_energy += current_pass.getLength()#current_pass.getEnergy(current_pass_config,self.routemanager)   # Add energy required to traverse path

                if index+1 < self.routeSize():                                              # If not at the end of the route
                    destination_pass,destination_pass_config = self.getImagePass(index+1)   # Store next pass as destination pass
                else:
                    destination_pass,destination_pass_config = self.getImagePass(0)         # Make destination pass the first pass in list to create link

                # Obtain energy and the dubins path required to connect the passes
                energy,dpath = current_pass.energyTo(current_pass_config,destination_pass,destination_pass_config,self.routemanager)
                if dpath is not None:
                    self.dubins_paths.append(dpath) # Add the shortest dubins path between the passes
                route_energy += energy              # Add calculated energy to link passes
            self.energy = route_energy              # Store calculated energy for this route
        return self.energy
    
    def getLength(self):
        """
        Add so that the entire route length can be calculated
        """
        length = 0
        for image_pass in self.route:
            length += image_pass[0].getLength()
        for dubins_path in self.dubins_paths:
            length += dubins_path.length()
        self.length = length
        return self.length


    def getDPaths(self):
        """
        Added by Thomas Pound
        Get the calculated shortest dubins paths to link the passes
        """
        return self.dubins_paths

    def routeSize(self):
        return len(self.route)

    def containsPass(self, image_pass):
        routeNp = np.array(self.route)
        if len(np.where(routeNp[:,0] == image_pass)[0]) > 0:
            return True
        else:
            return False 


    def orderPasses(self,index):
        """
        Added by Thomas Pound
        This function orders the passes so that the start location is first
        """
        self.route = np.roll(self.route,-index,axis=0)


class Population:
    """
    Population class saves all routes in the population
    """
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
    """
    Genetic algorithm class
    """
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

                route.setImagePass(routePos2,image_pass1,random.choice([True,False]))
                route.setImagePass(routePos1,image_pass2,random.choice([True,False]))
                
    def tournamentSelection(self, population):
        tournament = Population(self.routemanager, self.tournamentSize, False)
        for i in range(0,self.tournamentSize):
            randomId = int(random.random() * population.populationSize())
            tournament.saveRoute(i,population.getRoute(randomId))
        fittest = tournament.getFittest()
        return fittest

def TSP(image_passes,wind_angle,min_turn,uav_mass,NFZs,NFZ_edges,max_grad,glide_slope,
        start_loc,populationSize=50,mutationRate=0.015,generations=50,tournamentSize=20):
    """
    Travelling salesman problem
    parameters:
        image_passes    - All image passes on the route
        wind_angle      - Angle of the wind
        min_turn        - Min turn radius in metres for the aircraft
        uav_mass        - Mass of the aircraft in Kg
        NFZs            - Vertices of the NFZs
        max_grad        - Maximum incline gradient of the aircraft
        start_loc       - Coordinates of the takeoff and landing location
        population_size - How large the population of routes should be, default is 50
        mutation_rate   - How often/likely are the routes to mutate, default is 0.015
        generations     - How many evolutions should happen, default is 50
        tournament_size - How large should the tournament size be, default is 20

    returns:
        best_route      - The shortest path found from the GA TSP
    """

    # Create a pass for the start location
    start_pass = Image_Pass(start_loc,start_loc,start_loc[2],wind_angle)
    image_passes.append(start_pass)

    routemanager = RouteManager()   # Create a new route manager
    routemanager.setParams(wind_angle,min_turn,uav_mass,NFZs,NFZ_edges,math.radians(max_grad),
                            math.radians(glide_slope),start_loc)  # Set all parameters and settings
    
    # Add all passes to the routemanagers list
    routemanager.addImagePasses(image_passes)

    pop = Population(routemanager,populationSize,True)  # Create a population

    # Evolve population for the specified number of generations
    ga = GA(routemanager,mutationRate,tournamentSize)
    pop = ga.evolvePopulation(pop)

    for i in range(0, generations): # Evolve for the set number of generations
        pop = ga.evolvePopulation(pop)
        print(f"{100 * i/generations} %")   # Print the percentage of completion

    bestRoute = pop.getFittest()    # Get best route

    index = np.where(np.array(bestRoute)[:,0] == start_pass)[0][0]  # Find the start location in the route
    bestRoute.orderPasses(index)    # Order the passes so that the start location is first in the list

    return bestRoute


if __name__ == '__main__':
    """
    Test case with 4 define pass locations
    """
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