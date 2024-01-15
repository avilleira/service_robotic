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

# GLOBAL VARIABLES
obstacle_arr = []
dimensions = [0, 0, IMAGE_W, IMAGE_H]
map = cv.imread("map.png")

def isStateValid(state):
  x = round(state.getX())
  y = round(state.getY())
  #for i in range(len(obstacle_arr)):
  #  if sqrt(pow(x - obstacle_arr[i][1], 2) + pow(y - obstacle_arr[i][0], 2)) <= 0:
  #    return False
  if [y, x] not in obstacle_arr:
    return True
  return False

# Get obstacles array, thick them to avoid collisions
def get_obstacles(world_map):
  for row in range(IMAGE_H):
    for col in range(IMAGE_W):
      if np.any(world_map[row, col, :3]) == 0. and (row > 0 and row < IMAGE_H) and (col > 0 and col < IMAGE_W):
        for r in range(row - 5, row + 6): # With 8 works
          for c in range(col - 5, col + 6):
            if [r, c] not in obstacle_arr and r < IMAGE_H  and r >= 0 and c < IMAGE_W and c >= 0:
              obstacle_arr.append([r, c])

  for row in range(IMAGE_H):
    for col in range(IMAGE_W):
      if [row, col] in obstacle_arr:
        world_map[row, col] = [0, 0, 0]

# WORLD 2 MAP coordinates conversion
def world_2_map(coordx, coordy):
  
  new_col = round(20.0775945683802*coordx + 207)
  new_row = round(-20.4411764705882*coordy + 139)
  
  return new_col, new_row

def plan():
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
  start().setX((IMAGE_W-1)//2)
  start().setY(278//2)
  start().setYaw(math.pi / 2)
  goal = ob.State(space)
  goal().setX(195)
  goal().setY(60)
  goal().setYaw(math.pi/2)

  # create a problem instance
  pdef = ob.ProblemDefinition(si)

  # set the start and goal states
  pdef.setStartAndGoalStates(start, goal)

  # create a planner for the defined space
  planner = og.RRTConnect(si)
  # set the problem we are trying to solve for the planner
  planner.setProblemDefinition(pdef)

  # perform setup steps for the planner
  planner.setup()

  # solve the problem and print the solution if exists
  solved = planner.solve(1.0)
  if solved:
    print(pdef.getSolutionPath().printAsMatrix())
    path = create_numpy_path(pdef.getSolutionPath().printAsMatrix())
    for p in path:
      map[round(p[1]), round(p[0])] = [0, 0, 255]
  else:
    print("NOT FOUND")
  
def create_numpy_path(states):
  lines = states.splitlines()
  length = len(lines) - 1
  array = np.zeros((length, 2))
  for i in range(length):
      array[i][0] = float(lines[i].split(" ")[0])
      array[i][1] = float(lines[i].split(" ")[1])
  return array

get_obstacles(map)
map[70, 195] = [0, 255, 0]
plan()
cv.imshow("Map", map)
k = cv.waitKey(0)
