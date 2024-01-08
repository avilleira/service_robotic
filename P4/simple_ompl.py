from ompl import base as ob
from ompl import geometric as og
import math
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt

# specify valid state condition
def isStateValid(state):
  x = state.getX()
  y = state.getY()
  if sqrt(pow(x - obstacle[0], 2) + pow(y - obstacle[1], 2)) - obstacle[2] <= 0:
    return False
  return True

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
  start().setX(0)
  start().setY(0)
  start().setYaw(math.pi / 4)
  goal = ob.State(space)
  goal().setX(10)
  goal().setY(10)
  goal().setYaw(math.pi / 4)

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
    print(pdef.getSolutionPath())
    plot_path(pdef.getSolutionPath(), dimensions)

def create_numpy_path(states):
    lines = states.splitlines()
    length = len(lines) - 1
    array = np.zeros((length, 2))

    for i in range(length):
        array[i][0] = float(lines[i].split(" ")[0])
        array[i][1] = float(lines[i].split(" ")[1])
    return array

def plot_path(solution_path, dimensions):
  matrix = solution_path.printAsMatrix()
  path = create_numpy_path(matrix)
  x, y = path.T
  ax = plt.gca()
  ax.plot(x, y, 'r--')
  ax.plot(x, y, 'go') 
  ax.axis(xmin=dimensions[0], xmax=dimensions[2], ymin=dimensions[1], ymax=dimensions[3])
  ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), radius=obstacle[2]))

  plt.show()

if __name__ == "__main__":
  dimensions = [0, 0, 10, 10] 
  obstacle = [5.5, 5.5, 1]   # [x, y, radius]
  plan()