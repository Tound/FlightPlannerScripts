#Create Terrain
import matplotlib.pyplot as plt
import numpy as np
import math
from opensimplex import OpenSimplex
import random

height = 750
width = 750
freq = 2
octaves = (0.1,0.5,1)

gen = OpenSimplex(seed=random.randint(0,100))
def noise(nx, ny):
    # Rescale from -1.0:+1.0 to 0.0:1.0
    return gen.noise2d(nx, ny) / 2.0 + 0.5

value = np.zeros((width,height))
for y in range(height):
    for x in range(width):
        nx = x/width - 0.5
        ny = y/height - 0.5
        elevation = 20*noise(freq*nx, freq*ny)
        #+ 1/octaves[0]*noise(1/octaves[0] * nx, 1/octaves[0] * ny) 
        #+ 1/octaves[1]*noise(1/octaves[1] * nx, 1/octaves[1] * ny) 
        #+ 1/octaves[2]*noise(1/octaves[2] * nx, 1/octaves[2] * ny)
        value[y][x] =  10*math.pow(elevation,0.5)

fig = plt.figure(num=1,clear=True,figsize=(12,8))
ax = fig.add_subplot(1,1,1,projection='3d')

(x,y) = np.meshgrid(np.arange(0,width,1),np.arange(0,height,1))
ax.plot_surface(x, y, value,cmap='terrain')
ax.set(title='Terrain Generated',xlabel='x', ylabel='y', zlabel='z = Height (m)')
ax.set_zlim(0,150)

#ax.set_aspect(aspect='auto')
fig.tight_layout()

#plt.imshow(value,cmap='terrain')
plt.show()

