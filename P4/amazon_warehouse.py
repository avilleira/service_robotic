from GUI import GUI
from HAL import HAL
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
import math
import numpy as np
import time

# CONSTANTS

IMAGE_H = 279
IMAGE_W = 415
SHELF_POS  = [3.728, 0.579]
OBSTACLE_SIZE = 3
DISTANCE_ERR = 0.1

# STATE 
MOVE_TO = 0
SHELF_POS = 1
RETURN = 2
FINISHED = 3

# GLOBAL VARIABLES

obstacle_arr = []
dimensions = [0, 0, IMAGE_H, IMAGE_W]
map = GUI.getMap('/RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png')

def isStateValid(state):

  x = round(state.getX())
  y = round(state.getY())

  if [x, y] not in obstacle_arr:
    for i in range(len(obstacle_arr)):
      if sqrt(pow(x - obstacle_arr[i][0], 2) + pow(y - obstacle_arr[i][1], 2)) - 4 <= 0:
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
  start_posx, start_posy = world_2_map(HAL.getPose3d().x, HAL.getPose3d().y)
  start().setX(start_posx)
  start().setY(start_posy)
  start().setYaw(HAL.getPose3d().yaw)
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
  #Set max distance range between states
  planner.setRange(30)
  # set the problem we are trying to solve for the planner
  planner.setProblemDefinition(pdef)

  # perform setup steps for the planner
  planner.setup()

  # solve the problem and print the solution if exists
  solved = planner.solve(20.0)
  if solved:
    path = create_numpy_path(pdef.getSolutionPath().printAsMatrix())
    GUI.showPath(path)
    return path
  else:
    print("NOT FOUND")
  
def create_numpy_path(states):
  lines = states.splitlines()
  length = len(lines) - 1
  array = np.zeros((length, 2))
  for i in range(length):
      array[i][1] = round(float(lines[i].split(" ")[0]))
      array[i][0] = round(float(lines[i].split(" ")[1]))
  return array


#----------------- MOTION FUNCTIONS -----------------

def angular_vel(setpoint, prev_err):
  Kp = 0.7
  Kd = 0.35
  
  setpoint_posx, setpoint_posy = map_2_world(setpoint[1], setpoint[0])
  set_angle = math.atan2(setpoint_posy - HAL.getPose3d().y, setpoint_posx - HAL.getPose3d().x)
  curr_error = set_angle - HAL.getPose3d().yaw
  print(curr_error)
  w = Kp*curr_error + Kd*(curr_error - prev_err)
  return w, curr_error



get_obstacles(map)
print("Mapa construido")
dest_x, dest_y = world_2_map(3.728, 0.579)
path_arr = plan(dest_x, dest_y)
print(path_arr)

# Enter sequential code!

state = MOVE_TO
path_index = 1
err = 0
while True:
    # Enter iterative code!
    if state == MOVE_TO:
      HAL.setV(0.05)
      ang_v, err = angular_vel(path_arr[path_index], err)
      HAL.setW(ang_v)
      path_x, path_y = map_2_world(path_arr[path_index][1], path_arr[path_index][0])
      
      if (abs(path_x - HAL.getPose3d().x) <= DISTANCE_ERR and
        abs(path_y - HAL.getPose3d().y) <= DISTANCE_ERR):
          path_index += 1
          
      if path_index == len(path_arr):
        state = SHELF_POS
      
    elif state == SHELF_POS:
      HAL.setV(0)
      
      if round(HAL.getPose3d().yaw - math.pi) != 0:
        HAL.setW(0.1)
        
      else:
        HAL.setW(0)
        time.sleep(1)
        HAL.lift()
        state = RETURN
        path_index = 0
        err = 0
        
    elif state == RETURN:
      
      HAL.setV(0.05)
      ang_v, err = angular_vel(path_arr[path_index], err)
      HAL.setW(0.014)
      print()
      path_x, path_y = map_2_world(path_arr[path_index][1], path_arr[path_index][0])
      
      if (abs(path_x - HAL.getPose3d().x) <= DISTANCE_ERR and
        abs(path_y - HAL.getPose3d().y) <= DISTANCE_ERR):
          path_index -= 1
          
      if path_index == -1:
        HAL.putdown()
        HAL.setv(0)
        HAL.setW(0)
        state = FINISHED
    