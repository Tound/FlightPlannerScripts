import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as path
from dubinspython import *
#from cam_script import *
import matplotlib.patches as patches

points = np.array([(100,100),(200,300),(200,0),(100,0),(0,200),(0,-100)])
wind = (5,-45)

x_points = np.array([200,800,2000,2500,3000,3000,1500,500])
y_points = np.array([1000,2500,3000,2500,1500,100,-800,-500])

x_points = np.append(x_points,x_points[0]) # Add first point again to link polygon
y_points = np.append(y_points,y_points[0]) # Add first point again to link polygon

edges = []
ppoints = []
for i in range(0,len(x_points)-1):
    ppoints.append((x_points[i],y_points[i]))
    edge = [(x_points[i],y_points[i]),(x_points[i+1],y_points[i+1])]
    edges.append(edge)


path = path.Path(ppoints)
print(edges)

sensor_x = 58.42
sensor_y = 25.4
focal_length = 16.5
aspect_ratio = "16:9"

sensor_footprint = (sensor_x,sensor_y)
side_overlap = 20
forward_overlap = 60

image_width = 4000 #px
image_height = 3000 #px

resolution = 50.8 #mm per pixel

coverage_x = image_width*resolution*10**-3
coverage_y = image_height*resolution*10**-3

print(coverage_x,coverage_y)

altitude = focal_length*coverage_x/sensor_x

print(altitude)

wind_angle = math.radians(wind[1])

print(math.cos(wind_angle),math.sin(wind_angle))
dx = math.cos(wind_angle)
dy = math.sin(wind_angle)

grad = dy/dx

inv_grad = -1/grad # Perpendicular to the wind

max_val = -math.inf
min_val = math.inf
for i in range(0, len(x_points)-1):

    val = y_points[i] - x_points[i]*inv_grad 
    if val > max_val:
        max_point = (x_points[i],y_points[i])
        max_val = val
    if val < min_val:
        min_point = (x_points[i],y_points[i])
        min_val = val

print(max_point, min_point)

#Find entire length of section
x_intercept = (min_val-max_val)/(grad-inv_grad)
y_intercept = inv_grad*x_intercept + min_val
dx = x_intercept
dy = max_val - y_intercept
length = math.sqrt(dx*dx + dy*dy)
print(length)

#x_axis = np.arange(0,dx+10,10)
#plt.xlim(-1000,3000)
#plt.ylim(-1000,3000)
#plt.plot(0,max_val,'o')
#plt.plot(x_intercept,y_intercept,'o')
#plt.plot(x_axis,x_axis*grad + max_val,'o')

x_axis = np.arange(0,3000,10)

min_y_axis = np.array([])
max_y_axis = np.array([])
for i in range(0,len(x_axis)):
    min_y_axis = np.append(min_y_axis,x_axis[i]*inv_grad + min_val) 
    max_y_axis = np.append(max_y_axis,x_axis[i]*inv_grad + max_val)


# Find center point of polygon
x_mean = 0
y_mean = 0
for i in range(0,len(x_points)-1):
    x_mean += x_points[i]
    y_mean += y_points[i]

center_point = (x_mean/(len(x_points)-1),y_mean/(len(y_points)-1))

# Find number of passses required
distance_between_photos_x = coverage_x - coverage_x*side_overlap/100
distance_between_photos_y = coverage_y - coverage_y*forward_overlap/100
passes = (length-coverage_x)/distance_between_photos_x + 1
val_diff = max_val-min_val
conv_factor = val_diff/length
pass_coords = [conv_factor*coverage_x/2]

y_shift = min_val+conv_factor*coverage_x/2


def ray_tracing(x,y):
    inside = False

    for edge in edges:
        A, B = edge[0],edge[1]
        if (y > B[1] or y < A[1] or x > max(A[0], B[0])):
            # The horizontal ray does not intersect with the edge
            continue

        if A[1] > B[1]:
            A, B = B, A

        # Make sure point is not at same height as vertex
        if y == A[1] or y == B[1]:
            y += 0.000001

        if x < min(A[0], B[0]): # The ray intersects with the edge
            inside = not inside
            continue

        try:
            m_edge = (B[1] - A[1]) / (B[0] - A[0])
        except ZeroDivisionError:
            m_edge = math.inf

        try:
            m_point = (y - A[1]) / (x - A[0])
        except ZeroDivisionError:
            m_point = math.inf

        if m_point >= m_edge:
            # The ray intersects with the edge
            inside = not inside
            continue
    return inside

total_pass_points = []
for i in range(0,int(passes)-1):
    y_shift += conv_factor*distance_between_photos_x
    new_x_axis = np.array([])
    pass_points = []
    for j in range(0,len(x_axis)):
        y = inv_grad*x_axis[j] + y_shift
        if path.contains_point((x_axis[j],y)):
        #if ray_tracing(x_axis[j],y):
            new_x_axis = np.append(new_x_axis,x_axis[j])
            pass_points.append((x_axis[j],y))
    total_pass_points.append(pass_points)
    plt.plot(new_x_axis,inv_grad*new_x_axis + y_shift, color= 'red')


min_turn = 25
angle = wind_angle+math.pi/2
paths = []
turn_x = []
turn_y = []
print(len(total_pass_points))
last = True # Chooses last point or not
for i in range(0,len(total_pass_points)-1):
    if last:
        current_pass = total_pass_points[i]
        end_x = current_pass[len(current_pass)-1][0]
        end_y = current_pass[len(current_pass)-1][1]
        q0 = (end_x,end_y,angle)
        next_pass = total_pass_points[i+1]
        end_x = next_pass[len(next_pass)-1][0]
        end_y = next_pass[len(next_pass)-1][1]
        angle += math.pi
        q1 = (end_x,end_y,angle)
        path = dubins_shortest_path(q0,q1,min_turn)
    else:
        current_pass = total_pass_points[i]
        start_x = current_pass[0][0]
        start_y = current_pass[0][1]
        q0 = (start_x,start_y,angle)
        next_pass = total_pass_points[i+1]
        start_x = next_pass[0][0]
        start_y = next_pass[0][1]
        angle -= math.pi
        q1 = (start_x,start_y,angle)
        path = dubins_shortest_path(q0,q1,min_turn)

    turn_points = dubins_path_sample_many(path,5)
    for point in turn_points:
        turn_x.append(point[0])
        turn_y.append(point[1])
    last = not last

theta = wind_angle-math.pi/2
for pass_ in total_pass_points:
    start = (pass_[0][0],pass_[0][1])
    end = (pass_[len(pass_)-1][0],pass_[len(pass_)-1][1])

    pass_length = math.sqrt((pass_[len(pass_)-1][0]-pass_[0][0])**2 + (pass_[len(pass_)-1][1]-pass_[0][1])**2)

    spots = pass_length/distance_between_photos_y

    x = coverage_y
    while x < pass_length:
        photo_coords_x, photo_coords_y = start[0] - x*math.cos(theta), start[1] - x*math.sin(theta)
        plt.plot(photo_coords_x,photo_coords_y,'o',markersize='3',color='blue')
        dy = coverage_y/2 * math.sin(theta)
        dx = coverage_y/2 * math.cos(theta)
        new_angle = math.pi+theta
        dy2 = coverage_x/2 * math.sin(new_angle)
        dx2 = coverage_x/2 * math.cos(new_angle)
        rect_x, rect_y = photo_coords_x - dx-dx2, photo_coords_y-dy+dy2
        rectangle = plt.Rectangle((rect_x,rect_y),coverage_x,coverage_y,angle=math.degrees(wind_angle),fill=False,color="red")
        plt.gca().add_patch(rectangle)
        x += distance_between_photos_y
    



plt.title("2D coverage test with Dubins test")
plt.xlabel("X distance")
plt.ylabel("Y distance")

plt.plot(center_point[0],center_point[1],marker='o')
plt.plot(x_points,y_points,marker='o',linestyle= 'solid')
plt.plot(turn_x,turn_y,marker='o',linestyle= 'solid', markersize=1)
plt.plot(min_point[0],min_point[1],marker='o')
plt.plot(max_point[0],max_point[1],marker='o')
#plt.plot(x_axis,min_y_axis)
#plt.plot(x_axis,max_y_axis)

#print(ray_tracing(center_point[0],center_point[1]))
rectangle = plt.Rectangle((500,500),1000,500,angle=90,fill=False,color="red")
plt.plot(500,500,marker='o')
plt.gca().add_patch(rectangle)

plt.show()