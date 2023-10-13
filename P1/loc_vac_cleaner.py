from GUI import GUI
from HAL import HAL
import numpy as np
import math
import cv2

# Enter sequential code!
map = GUI.getMap('/RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc_newmanager/resources/mapgrannyannie.png')

def map_conversion (old_map):
  # The old map is a RGBA np.ndarray
  # If the pixel is white [1. 1. 1. 1.] and if it's black [0. 0. 0. 1.]
  new_map = np.zeros((old_map.shape[0], old_map.shape[1]))
  GUI.showNumpy(new_map)
  print(new_map)
  
  

def parse_laser_data(laser_data):
  laser = []
  for i in range(180):
    dist = laser_data.values[i]
    angle = math.radians(i)
    laser += [(dist, angle)]
  return laser
    

def laser_vector(laser):
  laser_vectorized = []
  for d,a in laser:
    # (4.2.1) laser into GUI reference system
    x = d * math.cos(a) * -1
    y = d * math.sin(a) * -1
    v = (x,y)
    laser_vectorized += [v]
    laser_mean = np.mean(laser_vectorized, axis=0)
  return laser_mean
  
  
map_conversion(map)
  
while True:
    # Enter iterative code!
    HAL.setV(1)