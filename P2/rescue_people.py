from GUI import GUI
from HAL import HAL
import math
import time


# ----------- GLOBAL VARIABLES ------------------
UTM_INIT = [430492, 4459162] # UTM: East and North
UTM_SURV_POSE = [430532, 4459132]

# STATES
TAKE_OFF = 0
PATROLLING = 1

#DATA
HEIGHT = 2 #m
SURVIVORS = 6
TIME_LIMIT = 600 # 10 min
SQUARE_AREA = 17 #m
TURN_LENGTH = 1

# DIRECTIONS
SOUTH = 1
EAST = 2
WEST = 3

def mv_survivors_init_pose ():
  # Calculating survivors position using UTM
  east_dest = UTM_SURV_POSE[0] - UTM_INIT[0]
  north_dest = UTM_SURV_POSE[1] - UTM_INIT[1]
  current_pos = HAL.get_position()
  
  # Move drone to the initial pose of the survivors:
  HAL.set_cmd_pos(east_dest, north_dest, 2, 0)
  
  while (round(current_pos[0]) != east_dest) and (round(current_pos[1]) != north_dest):
    current_pos = HAL.get_position()
    GUI.showLeftImage(HAL.get_ventral_image())
    GUI.showImage(HAL.get_frontal_image())
  

def rotate_yaw (angle):
  prev_pos = HAL.get_position()
  # Turn in yaw axe 
  pos = HAL.get_position()
  while (round(current_pos[2]) != angle):
    HAL.set_cmd_pos(pos[0], pos[1], HEIGHT, angle)
    GUI.showLeftImage(HAL.get_ventral_image())
    GUI.showImage(HAL.get_frontal_image())

  
# Enter sequential code!

state = TAKE_OFF
direction = WEST
prev_dir = 0
posx = 0
posy = 0

#Init time MAX 10 min
init_time = time.time()
while True:
    # Enter iterative code
    # Taking off state
    if state == TAKE_OFF:
      HAL.takeoff(HEIGHT)
      mv_survivors_init_pose()
      init_square_point = HAL.get_position()
      print("SALGO DE TAKE_OFF")
      state = PATROLLING
      
    elif state == PATROLLING:
      current_pos = HAL.get_position()
      speedx = 0
      speedy = 0
      # Changing direnctions
      if direction == SOUTH:
        speedx = 0
        speedy = -0.5
        
        if current_pos[1] <= posy - TURN_LENGTH:
          if prev_dir == WEST:
            direction = EAST
          elif prev_dir == EAST:
            direction = WEST
            
      elif direction == WEST:
        speedx = -1.5
        speedy = 0
        
        if current_pos[0] <= init_square_point[0] - init_square_point:
          prev_dir = direction
          direction = SOUTH
          posy = HAL.get_position()[1]
        
      
      elif direction == EAST:
        speedx = 1.5
        speedy = 0
        
        if current_pos[0] >= init_square_point[0] + 2:
          prev_dir = direction
          direction = SOUTH
          posy = HAL.get_position()[1]
      
      HAL.set_cmd_mix(speedx, speedy, HEIGHT, 0)  

    GUI.showLeftImage(HAL.get_ventral_image())
    GUI.showImage(HAL.get_frontal_image())