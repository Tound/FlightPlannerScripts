# Create Passes
import math
import numpy as np
import shapely.geometry as sg
from dubins_3D import *
from scipy.interpolate import griddata

G = 9.81

class Image_Pass:
    def __init__(self,image_locs,wind_angle):
        #self.id = id
        self.wind_angle = wind_angle
        #self.start = start
        #self.end = end
        self.image_locs = image_locs
        #image_locs.reverse()
        #self.image_locs_rev = image_locs
        self.energy = [None,None]
        self.length = None
        self.neighbours = None
        self.heading = [None,None]

    def getStart(self,config):
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
            for i,image_loc in enumerate(self.image_locs):
                dx = image_loc.x - self.image_locs[i-1].x
                dy = image_loc.y - self.image_locs[i-1].y
                dz = image_loc.z - self.image_locs[i-1].z
                length += math.sqrt(dx*dx + dy*dy + dz*dz)
            self.length = length
        return self.length

    def getEnergy(self,config,uav_mass):
        if config:
            if self.energy[0] is None:
                energy = 0
                for i,image_loc in enumerate(self.image_locs):
                    dx = image_loc.x - self.image_locs[i-1].x
                    dy = image_loc.y - self.image_locs[i-1].y
                    dz = image_loc.altitude - self.image_locs[i-1].altitude
                    if dz>0:
                        gpe = uav_mass*G*dz
                        energy += math.sqrt(dx*dx +dy*dy + dz*dz) + gpe
                    else:
                        energy += math.sqrt(dx*dx +dy*dy)
                self.energy[0] = energy
            return self.energy[0]

        else:
            if self.energy[1] is None:
                energy = 0
                for i,image_loc in reversed(list(enumerate(self.image_locs,))):
                    dx = image_loc.x - self.image_locs[i-1].x
                    dy = image_loc.y - self.image_locs[i-1].y
                    dz = image_loc.altitude - self.image_locs[i-1].altitude
                    if dz>0:
                        gpe = uav_mass*G*dz
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
                #if self.heading[1] is None:
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

        #print(f"{q0},{q1}")
        #print(d_path.length())
        dz = end.altitude - start.altitude
        if dz > 0:
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
        self.heading = None

    def setHeading(self,heading):
        if heading >= 2*math.pi:
            heading -= 2*math.pi
        self.heading = heading

    def getHeading(self):
        return self.heading

class Edge:
    def __init__(self,x1,y1,x2,y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.edge = sg.LineString([(x1,y1),(x2,y2)])

    def getEdge(self):
        return self.edge

def getDistance(point1,point2):
    dx = point2[0]-point1[0]
    dy = point2[1]-point1[1]
    return math.sqrt(dy*dy + dx*dx)

def convertCoords(vertices,angle,coord_system):
    """
    Performs coordinate transformatin between uv <=> xy
    Using   [u] = [ cos(theta) + sin(theta)][x]
            [v]   [-sin(theta) + cos(theta)][y]
    """
    theta = angle
    new_coords = []
    # Matrix multiplication?
    if coord_system == 'uv':
        for vertex in vertices:
            u = vertex[0]*math.cos(theta) + vertex[1]*math.sin(theta)
            v = -vertex[0]*math.sin(theta) + vertex[1]*math.cos(theta)
            new_coord = [u,v]
            new_coords.append(new_coord)
    elif coord_system == 'xy':
        scaler = 1/(math.cos(theta) * math.cos(theta) + math.sin(theta) * math.sin(theta))
        for vertex in vertices:
            x = scaler*(vertex[0]*math.cos(theta) - vertex[1]*math.sin(theta))
            y = scaler*(vertex[0]*math.sin(theta) + vertex[1]*math.cos(theta))
            new_coord = [x,y]
            new_coords.append(new_coord)
    else:
        print("Unknown coord system - Choose either 'xy' or 'uv'")
    return new_coords

def createPasses(area,NFZs,terrain,config):
    camera = config.camera
    wind_angle = config.wind[1]

    image_passes = []

    # Update coordinate system
    new_area_coords = convertCoords(area,wind_angle,'uv')
    new_NFZs = []
    for NFZ in NFZs:
        new_NFZ_coords = convertCoords(NFZ,wind_angle,'uv')
        new_NFZs.append(new_NFZ_coords)

    # Find footprint size
    coverage_width = config.coverage_resolution * camera.image_x
    coverage_height = config.coverage_resolution * camera.image_y

    uav_altitude = coverage_width *camera.focal_length/camera.sensor_x

    distance_between_photos_width = coverage_width - coverage_width*config.side_overlap
    distance_between_photos_height = coverage_height - coverage_height*config.forward_overlap

    pass_angle = math.pi/2  # In new coord system

    # Obtain properties about the area
    sorted_vertices = sorted(new_area_coords, key=lambda u:u[0])
    length_of_area = sorted_vertices[len(sorted_vertices)-1][0] - sorted_vertices[0][0]
    print(f"{convertCoords([sorted_vertices[len(sorted_vertices)-1]],wind_angle,'xy')},{convertCoords([sorted_vertices[0]],wind_angle,'xy')}")

    np_area = np.array(new_area_coords)
    max_height = np.max(np_area[:,1])           # Store highest V value
    min_height = np.min(np_area[:,1])           # Store lowest V value
    start_u = np.min(np_area[:,0])

    # Calculate number of passes to cover area
    number_of_passes = (length_of_area-coverage_width)/distance_between_photos_width + 1

    polygon_edges = []
    NFZ_edges = []

    # Create all edges
    for i in range(0,len(new_area_coords)):
        polygon_edges.append(Edge(new_area_coords[i-1][0],new_area_coords[i-1][1],
                                    new_area_coords[i][0],new_area_coords[i][1]))
    
    for NFZ_coords in new_NFZs:
        for i in range(0,len(NFZ_coords)):
            NFZ_edges.append(Edge(NFZ_coords[i-1][0],NFZ_coords[i-1][1],
                                    NFZ_coords[i][0],NFZ_coords[i][1]))

    u = start_u
    for i in range(0,int(number_of_passes)):        # Cycle through all full-length passes across entirety of area
        u += distance_between_photos_width          # Increase U value on each loop

        subpasses = 1

        pass_edge = sg.LineString([(u,min_height-1),(u,max_height+1)])
        max_intersect = (-math.inf,-math.inf)
        min_intersect = (math.inf,math.inf)
        for edge in polygon_edges:
            intersection = edge.getEdge().intersection(pass_edge)
            #print(edge.getEdge(),pass_edge,intersection)
            if intersection.is_empty:
                continue
            if intersection.y >= max_intersect[1]:               # Even though we are using uv axis, shapely requires xy
                max_intersect = [intersection.x,intersection.y]
            if intersection.y <= min_intersect[1]:
                min_intersect = [intersection.x,intersection.y]
        pass_start = min_intersect
        pass_end = max_intersect
        total_pass_length = getDistance(min_intersect,max_intersect)

        # Update pass edge?
        # Check if pass crosses any NFZ edges
        

        # FIX TO ALLOW FOR MULTIPLE NFZs
        intersection_points = []
        for edge in NFZ_edges:        # Cycle through all NFZs
            intersection = edge.getEdge().intersection(pass_edge)
            if intersection.is_empty:
                continue
            else:
                intersection_points.append([intersection.x,intersection.y])

        points_on_pass = [min_intersect,max_intersect]

        if not len(intersection_points)==0:
            for point in intersection_points:
                points_on_pass.append(point)

        points_on_pass = sorted(points_on_pass,key=lambda point:point[1])   # List vertically
        #print(points_on_pass)
        subpasses = len(points_on_pass)/2

        # Should center the images so there are no large gaps
        for j in range(0,int(subpasses)):
            image_locations = []
            start = points_on_pass[j*2]
            end = points_on_pass[j*2 + 1]
            #print(start)
            #print(end)
            #print("\n")
            pass_length = getDistance(start,end)
            number_of_image_locations = (pass_length-coverage_height)/distance_between_photos_height
            if number_of_image_locations == 0:
                continue
            # Need to calculate overhang of image footprint

            if config.forward_overlap < 0.5:
                # Add extra image location to account for incomplete coverage on corners
                number_of_image_locations += 1
            
            v = start[1]
            for j in range(0,int(number_of_image_locations)):
                v += distance_between_photos_height
                coord = convertCoords([[u,v]],wind_angle,'xy')
                x = coord[0][0]
                y = coord[0][1]
                x_points = [int(x),int(x),int(x)+1,int(x)+1]
                y_points = [int(y),int(y)+1,int(y)+1,int(y)]
                z_points = [terrain[int(y)][int(x)],terrain[int(y)+1][int(x)],
                            terrain[int(y)+1][int(x)+1],terrain[int(y)][int(x)+1]]

                z = griddata((x_points,y_points),z_points,(x,y))    # Interpolate

                # Could average out the altiutude here>
                # Could run terraces?
                altitude = z + uav_altitude
                image_locations.append(Image_Location(x,y,altitude))

            # Center the image locations?
            if len(image_locations) > 0:
                start_xy = convertCoords([start],wind_angle,'xy')
                end_xy = convertCoords([end],wind_angle,'xy')
                image_passes.append(Image_Pass(image_locations,config.wind[1]))

    print(f"length = {length_of_area}")
    print(coverage_width,coverage_height)
    print(distance_between_photos_width,distance_between_photos_height)
    print(f"number_of_passes = {number_of_passes}")

    return image_passes

