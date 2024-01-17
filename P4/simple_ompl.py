import numpy as np
import cv2 as cv
from ompl import base as ob
from ompl import geometric as og
import math
from math import sqrt
import matplotlib.pyplot as plt



# Enter sequential code!

IMAGE_H = 279
IMAGE_W = 415
SHELF_POS  = [3.728, 0.579]
INIT_POS = [0, 0]
OBSTACLE_SIZE = 4

# GLOBAL VARIABLES
obstacle_arr = []
dimensions = [0, 0, IMAGE_H, IMAGE_W]
map = cv.imread("map.png")

def isStateValid(state):

  x = round(state.getX())
  y = round(state.getY())
  if [x, y] not in obstacle_arr:
    for i in range(len(obstacle_arr)):
      if sqrt(pow(x - obstacle_arr[i][1], 2) + pow(y - obstacle_arr[i][0], 2)) - 2 <= 0:
        return False
    return True
  return False

# Get obstacles array, thick them to avoid collisions
def get_obstacles(world_map):
  for row in range(IMAGE_H):
    for col in range(IMAGE_W):
      if 0 < row < IMAGE_H and 0 < col < IMAGE_W and np.any(world_map[row, col, :3] == 0.):
        for r in range(row - OBSTACLE_SIZE, row + OBSTACLE_SIZE + 1): # With 8 works
          for c in range(col - OBSTACLE_SIZE, col + OBSTACLE_SIZE + 1):
            if [r, c] not in obstacle_arr and 0 <= r < IMAGE_H and 0 <= c < IMAGE_W :
              obstacle_arr.append([r, c])

  for row in range(IMAGE_H):
    for col in range(IMAGE_W):
      if [row, col] in obstacle_arr:
        world_map[row, col] = [0, 0, 0]

# WORLD 2 MAP coordinates conversion
# row is x and col is y
def world_2_map(coordx, coordy):

  new_row = round(-20.4411764705882*coordx + 139)
  new_col = round(-20.0775945683802*coordy + 207)
  
  return new_row, new_col

def map_2_world(row, col):
  posx = -0.0489208633094*row + 6.8
  posy = -0.049806763285*col + 10.31

  return posx, posy
def plan(destx, desty):
  # Construct the robot state space in which we're planning. We're
  # planning in [0,1]x[0,1], a subset of R^2.
  space = ob.SE2StateSpace()

  # set state space's lower and upper bounds
  bounds = ob.RealVectorBounds(2)
  bounds.setLow(0, dimensions[0])
  bounds.setLow(1, dimensions[1])
  bounds.setHigh(0, dimensions[2])
  bounds.setHigh(1, dimensions[3])
  space.setBounds(bounds)

  # construct a space information instance for this state space
  si = ob.SpaceInformation(space)
  # set state validity checking for this space
  si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

  # Set our robot's starting and goal state
  start = ob.State(space)
  start().setX((IMAGE_H-1)//2)
  start().setY((IMAGE_W-1)//2)
  start().setYaw(0)
  goal = ob.State(space)
  goal().setX(destx)
  goal().setY(desty)
  goal().setYaw(0)

  # create a problem instance
  pdef = ob.ProblemDefinition(si)

  # set the start and goal states
  pdef.setStartAndGoalStates(start, goal)

  # create a planner for the defined space
  planner = og.RRTConnect(si)
  # Set the number of space between spaces
  planner.setRange(30)
  # set the problem we are trying to solve for the planner
  planner.setProblemDefinition(pdef)

  # perform setup steps for the planner
  planner.setup()

  # solve the problem and print the solution if exists
  solved = planner.solve(20.0)
  if solved:
    print(pdef.getSolutionPath().printAsMatrix())
    path = create_numpy_path(pdef.getSolutionPath().printAsMatrix())
    print(path)
    for p in path:
      map[round(p[0]), round(p[1])] = [0, 0, 255]
  else:
    print("NOT FOUND")
  
def create_numpy_path(states):
  lines = states.splitlines()
  length = len(lines) - 1
  array = np.zeros((length, 2))
  for i in range(length):
      array[i][0] = round(float(lines[i].split(" ")[0]))
      array[i][1] = round(float(lines[i].split(" ")[1]))
  return array

h, y = map_2_world(IMAGE_H//2, IMAGE_W//2)
print(h, y)
get_obstacles(map)
coord_x, coord_y = world_2_map(3.728, 0.579)
dest_x, dest_y = world_2_map(3.728, 0.579)
h, y = map_2_world(IMAGE_H//2, IMAGE_W//2)
plan(dest_x, dest_y)
map[coord_x, coord_y] = [8, 255, 137]
cv.imshow("Amazon warehouse 1", map)
k = cv.waitKey(0)
