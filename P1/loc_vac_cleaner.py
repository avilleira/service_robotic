from GUI import GUI
from HAL import HAL
import numpy as np
import math
import cv2
import time

# Enter sequential code!

CELLSIZE = 17

class Cell:
  def __init__(self, cntr_x, cntr_y):
    self.h = 17
    self.w = 17
    self.centerX = cntr_x
    self.centerY = cntr_y
    self.obstacle = False
    self.cleaned = False
    
  def set_clean(self):
    self.cleaned = True
    
  def set_obstacle_cell(self):
    self.obstacle = True
    

def map_conversion (old_map):
  # The old map is a RGBA np.ndarray
  # If the pixel is white [1. 1. 1. 1.] and if it's black [0. 0. 0. 1.]
  # As the initial image is really big, resize it is an option to avoid computer cost
  r_map = cv2.resize(old_map, (512, 512))
  new_map = np.zeros((r_map.shape[0], r_map.shape[1]))
  
  for row in range(new_map.shape[0]):
    for col in range(new_map.shape[1]):
      if np.any(r_map[row, col, :3]) == 1.0:
        new_map[row, col] = 127
      else:
        new_map[row, col] = 128
        
  return new_map


# Creating mesh
def map_cells(raw_map):
  # We are gonna widen the edges
  # Creating cells
  row, col= raw_map.shape
  cells_arr = [[None] * (col // CELLSIZE) for _ in range(row // CELLSIZE)]
  cent_px = 8
  cent_py = 8
  # Creating all cells
  for i in range(row // CELLSIZE):
    cent_px = 8
    for j in range(col // CELLSIZE):
      cells_arr[i][j] = Cell(cent_px, cent_py)
      # Next col
      cent_px += 17
    # Next row
    cent_py += 17
  
  return cells_arr
    
"""This function will check if any pixel of the map is occupied with an obstacle, and
   it will put it in black and white"""
def is_obstacle(cell):
  c = cell.centerX - 8
  r = cell.centerY - 8
  
  for i in range(cell.h):
    for j in range(cell.w):
      if map[i + r,j + c] == 128:
        cell.set_obstacle_cell()
        break
    if map[i + r,j + c] == 128:
      break

def draw_obstacle(cell):
  c = cell.centerX - 8
  r = cell.centerY - 8
  
  for i in range(cell.h):
    for j in range(cell.w):
      map[i + r, j + c] = 128
      
def draw_cleaned(cell):
  c = cell.centerX - 8
  r = cell.centerY - 8
  
  for i in range(cell.h):
    for j in range(cell.w):
      map[i + r, j + c] = 132
  
  
def map_mesh():
  for row in range(len(cells)):
    for col in range(len(cells)):
      is_obstacle(cells[row][col])
      
      if (cells[row][col].obstacle == True):
        draw_obstacle(cells[row][col])
        
 
def create_grid():
  w, h = map.shape
  
  for j in range(0, w, CELLSIZE):
    cv2.line(map, (j, 0), (j, h), (0, 0, 0), 1)
  for i in range(0, w, CELLSIZE):
    cv2.line(map, (0, i), (w, i), (0, 0, 0), 1)
    
  

map = map_conversion(GUI.getMap('/RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc_newmanager/resources/mapgrannyannie.png'))
# Creating all cells
cells = map_cells(map)
map_mesh()
create_grid()
GUI.showNumpy(map)


while True:
  
    print("X:", HAL.getPose3d().x, "Y:", HAL.getPose3d().y)