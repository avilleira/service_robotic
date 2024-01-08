from GUI import GUI
from HAL import HAL
from ompl import base as ob
from ompl import geometric as og
import numpy as np
# Enter sequential code!

IMAGE_H = 279
IMAGE_W = 415

# Erode obstacle borders to avoid collision
def erode_map():
  
  for col in range(IMAGE_W):
    for row in range(IMAGE_H):
      if map_world[row, col].any == 0:
        if (row > 0 or row < IMAGE_H - 1) and (col > 0 or col < IMAGE_W - 1):
          map_world[row, col - 1] = [0., 0., 0.]
          map_world[row - 1, col - 1] = [0., 0., 0.]
          map_world[row - 1, col] = [0., 0., 0.]
          map_world[row - 1, col + 1] = [0., 0., 0.]
          map_world[row, col + 1] = [0., 0., 0.]
          map_world[row + 1, col + 1] = [0., 0., 0.]
          map_world[row + 1, col] = [0., 0., 0.]
          map_world[row + 1, col - 1] = [0., 0., 0.]

map_world = GUI.getMap('RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png')

erode_map()
while True:
    # Enter iterative code!