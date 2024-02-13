from GUI import GUI
from HAL import HAL
import numpy as np
import math
import cv2
import time


CELLSIZE = 17
BLUE = 128
RED = 132
WHITE = 127
YELLOW = 130

LINEAR_SPEED = 0.6
SLOW_LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.3
ORIENTATION_MAX_DIFF = 0.1
MIN_WORLD_DISTANCE = 0.4

OR_WEST = 0
OR_NORTH = - math.pi / 2
OR_EAST = -math.pi
OR_EAST_LEFT = math.pi
OR_SOUTH = math.pi / 2

class Cell:
  def __init__(self, cntr_x, cntr_y, r, c):
    self.row = r
    self.col = c
    self.centerX = cntr_x
    self.centerY = cntr_y
    self.obstacle = False
    self.cleaned = False
    # Relating all cells around it
    self.n_neighbor = 0
    self.e_neighbor = 0
    self.s_neighbor = 0
    self.w_neighbor = 0
    # Corners neighbors
    self.n_e_neighbor = 0
    self.e_s_neighbor = 0
    self.s_w_neighbor = 0
    self.w_n_neighbor = 0

   
  def set_clean(self):
    self.cleaned = True
   
  def set_obstacle_cell(self):
    self.obstacle = True
   
def map_conversion(old_map):
  # The old map is a RGBA np.ndarray
  # If the pixel is white [1. 1. 1. 1.] and if it's black [0. 0. 0. 1.]
  # As the initial image is really big, resize it is an option to avoid computer cost
  r_map = cv2.resize(old_map, (512, 512))
  new_map = np.zeros((r_map.shape[0], r_map.shape[1]))
 
  for row in range(new_map.shape[0]):
    for col in range(new_map.shape[1]):
      if np.any(r_map[row, col, :3]) == 1.0:
        new_map[row, col] = WHITE
      else:
        new_map[row, col] = RED
       
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
      cells_arr[i][j] = Cell(cent_px, cent_py, i, j)
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
 
  for i in range(CELLSIZE):
    for j in range(CELLSIZE):
      if map[i + r,j + c] == RED:
        cell.set_obstacle_cell()
        break
    if map[i + r,j + c] == RED:
      break

def draw_obstacle(cell):
  c = cell.centerX - 8
  r = cell.centerY - 8
 
  for i in range(CELLSIZE):
    for j in range(CELLSIZE):
      map[i + r, j + c] = BLUE
     
def draw_cleaned(cell):
  c = cell.centerX - 8
  r = cell.centerY - 8
 
  for i in range(CELLSIZE):
    for j in range(CELLSIZE):
      map[i + r, j + c] = RED

def draw_route_cell(cell):
  c = cell.centerX - 8
  r = cell.centerY - 8
 
  for i in range(CELLSIZE):
    for j in range(CELLSIZE):
      map[i + r, j + c] = YELLOW
     
def set_neighbors(cells):
  for row in range(len(cells)):
    for col in range(len(cells)):
      # we are not interested in the obstacles
      if (cells[row][col].obstacle == True):
        continue
      else:
        # NORTH NEIGHBOR
        cells[row][col].n_neighbor = cells[row - 1][col]
        # SOUTH NEIGHBOR
        cells[row][col].s_neighbor = cells[row + 1][col]
        # EAST NEIGHBOR
        cells[row][col].e_neighbor = cells[row][col + 1]
        # WEST NEIGHBOR
        cells[row][col].w_neighbor = cells[row][col - 1]
       
        # WEST NEIGHBOR
        cells[row][col].n_e_neighbor = cells[row - 1][col + 1]
        # WEST NEIGHBOR
        cells[row][col].e_s_neighbor = cells[row + 1][col + 1]
        # WEST NEIGHBOR
        cells[row][col].s_w_neighbor = cells[row + 1][col - 1]
        # WEST NEIGHBOR
        cells[row][col].w_n_neighbor = cells[row - 1][col - 1]
 
 
''' Stablish for all cells if it is an obstacle and save those which are not'''

def map_mesh():
  for row in range(len(cells)):
    for col in range(len(cells)):
      is_obstacle(cells[row][col])
     
      if (cells[row][col].obstacle == True):
        draw_obstacle(cells[row][col])
      else:
        free_cells.append(cells[row][col])
       
def create_grid():
  w, h = map.shape
 
  for j in range(0, w, CELLSIZE):
    cv2.line(map, (j, 0), (j, h), (0, 0, 0), 1)
  for i in range(0, w, CELLSIZE):
    cv2.line(map, (0, i), (w, i), (0, 0, 0), 1)
   
def pose_to_cell(x_coord, y_coord):
  # Linear regresion to transform GAZEBO coords to the create_grid
  x_cell = round(-2.9555517292899 * x_coord + 16.6514905522259)
  y_cell = round(3.0048943492781 * y_coord + 11.6877761275171)
  return cells[y_cell][x_cell]
 
def cell_to_pose(cell_x, cell_y):
  x_coord = (cell_x - 16.6514905522259) / (-2.9555517292899)
  y_coord = (cell_y - 11.6877761275171) / (3.0048943492781)
  #x_coord = -0.3377916820702 * cell_x + 5.6272593530499
  #y_coord = 0.3327115850537 * cell_y - 3.8882634703975
  return x_coord, y_coord

def save_cleaned_cells_nb(cell):
  # Save the cleaned cells neighbors
  neighbors = [cell.n_neighbor, cell.e_neighbor, cell.s_neighbor, cell.w_neighbor,
    cell.n_e_neighbor, cell.e_s_neighbor, cell.s_w_neighbor , cell.w_n_neighbor]

  for n in neighbors:
    if n in free_cells and not n.cleaned and n not in visited_cells_nb:
      visited_cells_nb.append(n)
  if cell in visited_cells_nb:
    visited_cells_nb.remove(cell)

def compute_distance(a, b):
  # Returns world distances between a and b
  distance = math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
  return distance
 
def distance_to(origin, dest):
  # We are going to calculate the distance between two positions
  dest_x, dest_y = cell_to_pose(dest.col, dest.row)
  ori_x, ori_y = cell_to_pose(origin.col, origin.row)
  distance = compute_distance([dest_x, dest_y], [ori_x, ori_y])
  return distance
 
def find_closest_cell(ref_cell, cells_list):
  # Look for the nearest cell from 'ref_cell' in the specified list
  dist = 1000
  for c in cells_list:
    if c in free_cells and distance_to(c, ref_cell) < dist:
      dist = distance_to(ref_cell, c)
      destiny = c
  draw_route_cell(destiny)
  return destiny, dist

def find_path(current_cell, destiny):
  # Find which neighbor is closer to destiny
  neighbors = [current_cell.n_neighbor, current_cell.e_neighbor, current_cell.s_neighbor, current_cell.w_neighbor]
  next_cell, _ = find_closest_cell(destiny, neighbors)

  # Change orientation so destiny is in front of robot
  current_cell = pose_to_cell(HAL.getPose3d().x, HAL.getPose3d().y)
  if current_cell.row > next_cell.row: # Destiny is the north neighbor
    desired_ori = OR_NORTH
  elif current_cell.row < next_cell.row: # Destiny is the south neighbor
    desired_ori = OR_SOUTH
  elif current_cell.col > next_cell.col: # Destiny is the west neighbor
    desired_ori = OR_WEST
  elif current_cell.col < next_cell.col: # Destiny is the east neighbor
    desired_ori = OR_EAST
  return next_cell, desired_ori

def set_state(orientation):
  if orientation == OR_NORTH:
    state = "NORTH"
  elif orientation == OR_EAST:
    state = "EAST"
  elif orientation == OR_SOUTH:
    state = "SOUTH"
  else:
    state = "WEST"
  return state

def compute_speed(curr_ori, desired_ori):
  right_turns = [[OR_WEST, OR_NORTH], [OR_NORTH, OR_EAST], [OR_EAST, OR_SOUTH], [OR_SOUTH, OR_WEST]]
  left_turns = [[OR_WEST, OR_SOUTH], [OR_SOUTH, OR_EAST], [OR_EAST, OR_NORTH], [OR_NORTH, OR_WEST]]
 
  if [curr_ori, desired_ori] in left_turns:
    return -ANGULAR_SPEED
  else:
    return ANGULAR_SPEED

# Sequential Code
map = map_conversion(GUI.getMap('/RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc_newmanager/resources/mapgrannyannie.png'))

# Creating all cells
cells = map_cells(map)
free_cells = [] # Cells without obstacles
visited_cells_nb = []
map_mesh()

cells[26][17].set_obstacle_cell()
cells[26][19].set_obstacle_cell()
cells[24][17].set_obstacle_cell()
draw_obstacle(cells[26][17])
draw_obstacle(cells[26][19])
draw_obstacle(cells[24][17])

set_neighbors(cells)
create_grid()
GUI.showNumpy(map)

# In order to reduce computational cost, we show the map every 10 iterations (Reducing to 10%)
iterations = 0

state = "WEST" # Robot initial orientation
desired_orientation = OR_WEST
turning = False

# Critical point handling
return_point = False
path = False
left_turn = False
while True:
    if iterations == 10:
      GUI.showNumpy(map)
      iterations = 0
   
    # Obtain current cell from pose and set it as cleaned
    current_cell = pose_to_cell(HAL.getPose3d().x, HAL.getPose3d().y)
    if not current_cell.cleaned:
      current_cell.set_clean()
      draw_cleaned(current_cell)
   
    if turning:
      dest_ori = desired_orientation
      if dest_ori == OR_EAST and not left_turn:
        diff = abs(HAL.getPose3d().yaw - dest_ori)
        if diff <= (ORIENTATION_MAX_DIFF + 0.05):
          HAL.setW(0) # Stop turning
          turning = False
      else:
        if left_turn:
          dest_ori = OR_EAST_LEFT
        diff = abs(HAL.getPose3d().yaw - dest_ori)
        if diff < ORIENTATION_MAX_DIFF:
          HAL.setW(0) # Stop turning
          turning = False
    else: # Not turning
      # Save neighbors of cleaned cells
      save_cleaned_cells_nb(current_cell)
     
      # CRITICAL POINT HANDLING
      if return_point:
        if not path:
          current_orientation = desired_orientation
          path, desired_orientation = find_path(current_cell, return_point)
          turning = True
          HAL.setV(0)
          speed = compute_speed(current_orientation, desired_orientation)
          if speed < 0:
            left_turn = True
          HAL.setW(speed)
        else:
          if current_cell == return_point:
            return_point = False
            HAL.setV(0)
            state = set_state(desired_orientation)
            HAL.setV(SLOW_LINEAR_SPEED)
            print("SET STATE DEVUELVE:", state)
          elif current_cell == path:
            HAL.setV(0)
            path = False
          else:
            HAL.setV(LINEAR_SPEED)
      else:
        critic_point = True
        neighbors = [current_cell.n_neighbor, current_cell.e_neighbor, current_cell.s_neighbor, current_cell.w_neighbor]
        for n in neighbors:
          if n in free_cells and not n.cleaned:
            critic_point = False
            break
        if critic_point:
          print("ARRIVED TO CRITIC POINT")
          return_point, _ = find_closest_cell(current_cell, visited_cells_nb)
          path = False
          state = None
     
      # BSA ALGORITHM State Machine
      if (state == "NORTH"):
        if (current_cell.n_neighbor.obstacle == False) and (current_cell.n_neighbor.cleaned == False):
          HAL.setV(LINEAR_SPEED)
        else:
          busy_cell_y = cell_to_pose(current_cell.n_neighbor.col, current_cell.n_neighbor.row)[1]
          if abs(HAL.getPose3d().y - busy_cell_y) < MIN_WORLD_DISTANCE: # Get close to obstacle
            desired_orientation = OR_EAST
            turning = True
            HAL.setV(0)
            HAL.setW(ANGULAR_SPEED)
            state = "EAST"
            print("STATE: EAST")
      elif (state == "EAST"):
        if (current_cell.e_neighbor.obstacle == False) and (current_cell.e_neighbor.cleaned == False):
          HAL.setV(LINEAR_SPEED)
        else:
          busy_cell_x = cell_to_pose(current_cell.e_neighbor.col, current_cell.e_neighbor.row)[0]
          if abs(HAL.getPose3d().x - busy_cell_x) < MIN_WORLD_DISTANCE: # Get close to obstacle
            desired_orientation = OR_SOUTH
            turning = True
            HAL.setV(0)
            HAL.setW(ANGULAR_SPEED)
            state = "SOUTH"
            print("STATE: SOUTH")
      elif (state == "SOUTH"):
        if (current_cell.s_neighbor.obstacle == False) and (current_cell.s_neighbor.cleaned == False):
          HAL.setV(LINEAR_SPEED)
        else:
          busy_cell_y = cell_to_pose(current_cell.s_neighbor.col, current_cell.s_neighbor.row)[1]
          if abs(HAL.getPose3d().y - busy_cell_y) < MIN_WORLD_DISTANCE: # Get close to obstacle
            desired_orientation = OR_WEST
            turning = True
            HAL.setV(0)
            HAL.setW(ANGULAR_SPEED)
            state = "WEST"
            print("STATE: WEST")
      elif state == "WEST":
        if (current_cell.w_neighbor.obstacle == False) and (current_cell.w_neighbor.cleaned == False):
          HAL.setV(LINEAR_SPEED)
        else:
          busy_cell_x = cell_to_pose(current_cell.w_neighbor.col, current_cell.w_neighbor.row)[0]
          if abs(HAL.getPose3d().x - busy_cell_x) < MIN_WORLD_DISTANCE: # Get close to obstacle
            desired_orientation = OR_NORTH
            turning = True
            HAL.setV(0)
            HAL.setW(ANGULAR_SPEED)
            state = "NORTH"
            print("STATE: NORTH")
    iterations += 1