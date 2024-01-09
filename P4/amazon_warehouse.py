from GUI import GUI
from HAL import HAL
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
import numpy as np
# Enter sequential code!

IMAGE_H = 279
IMAGE_W = 415
SHELF_POS  = [3.728, 0.579]

# GLOBAL VARIABLES
obstacle_arr = []
dimensions = [0, 0, IMAGE_H, IMAGE_W]

# Get obstacles array, thick them to avoid collisions
def get_obstacles(world_map):
  t = 0
  for row in range(IMAGE_H):
    for col in range(IMAGE_W):
      if np.any(world_map[row, col, :3]) == 0. and (row > 0 and row < IMAGE_H) and (col > 0 and col < IMAGE_W):
        for r in range(row - 1, row + 2):
          for c in range(col - 1, col + 2):
            if [r, c] not in obstacle_arr:
              obstacle_arr.append([r, c])

# WORLD 2 MAP coordinates conversion
def world_2_map(coordx, coordy):
  
  new_col = round(20.0775945683802*coordx + 207)
  new_row = round(-20.4411764705882*coordy + 139)
  
  return new_col, new_row


#--------------------------- PLANNING PART ---------------------------

def isStateValid(state):
  x = state.getX()
  y = state.getY()
  
  for obstacle in range(len(obstacle_arr)):
    if sqrt(pow(x - obstacle[0], 2) + pow(y - obstacle[1], 2)) - obstacle[2] <= 0:
      return False
  return True

def set_plan(current_pos):
  
  space = ob.SE2StateSpace() # Robot state space
  
  # Bounds
  bounds = ob.RealVectorBounds(2)
  bounds.setLow(0, dimensions[0])
  bounds.setLow(1, dimensions[1])
  bounds.setHigh(0, dimensions[2])
  bounds.setHigh(0, dimensions[3])
  space.setBounds(bounds)
  
  # construct space infromation
  space_info = ob.SpaceInformation(space)
  space_info.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
  
  # start and goal
  start_r, start_c = world_2_map(current_pos[0], current_pos[1])
  start = ob.State(space)
  start().setX(start_r)
  start().setY(start_c)
  start().setYaw(HAL.getPose3d().yaw)
  
  # Goal
  goal_r, goal_c = world_2_map(SHELF_POS[1], SHELF_POS[0])
  goal = ob.State(space)
  goal().setX(goal_r)
  goal().setY(goal_c)
  goal().setYaw(HAL.getPose3d().yaw)
  
  # Problem instance
  pdef = ob.ProblemDefinition(space_info)
  
  # set the start and goal states
  pdef.setStartAndGoalStates(start, goal)

  # create a planner for the defined space
  planner = og.RRTConnect(space_info)

  # set the problem we are trying to solve for the planner
  planner.setProblemDefinition(pdef)

  # perform setup steps for the planner
  planner.setup()

  # solve the problem and print the solution if exists
  print("VOY POR AQUI")
  solved = planner.solve(1.0)

  print("LA RUTA ES:", pdef.getSolutionPath())

  

  
map_world = GUI.getMap('RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png')

get_obstacles(map_world)

set_plan([0, 0])

while True:
    # Enter iterative code!