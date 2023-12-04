from GUI import GUI
from HAL import HAL
import math

# Enter sequential code!

MINDISTANCE = 5
# MACROS STATUS

SEARCHING = 0
ALIGNING = 1
PARKING = 2
PARKED = 3

# MOVEMENTS
LEFT_BACK = 0
BACK = 1
FRONT = 2
RIGHT_BACK = 3

front_right_laser = 60
back_right_laser = 120


def detect_free_space():
  r_laser = HAL.getRightLaserData().values

  free_space = False
  for angle in range(len(r_laser)):
    if angle >= front_right_laser and angle <= back_right_laser:
      if r_laser[angle] > HAL.getRightLaserData().maxRange:
        free_space = True
      else:
        free_space = False
        return free_space
  
  return free_space
  
  
def line_up_car():
  r_laser = HAL.getRightLaserData().values
  
  for angle in range(len(r_laser)):
    if angle >= 30 and angle <= 90:
      if r_laser[angle] > MINDISTANCE:
        return False
  return True
  
  
def distance_laser(laser):
  min_val = 100
  
  for angle_value in laser.values:
    if angle_value < min_val and angle_value > laser.minRange:
      min_val = angle_value
  
  return min_val
  
status = SEARCHING
movement = LEFT_BACK
left_back = False

init_ori = HAL.getPose3d().yaw

while True:
    # Enter iterative code!
    if status == SEARCHING:
      if detect_free_space() == True:
        HAL.setV(0)
        status = ALIGNING
      else:
        HAL.setV(1)
    
    elif status == ALIGNING:
      
      if line_up_car() == True:
        HAL.setV(0)
        status = PARKING
        # We keep current orientation
        init_ori = HAL.getPose3d().yaw
      else:
        HAL.setV(1)
        
    elif status == PARKING:
      if movement == LEFT_BACK:
        print(distance_laser(HAL.getBackLaserData()))
        if ((HAL.getPose3d().yaw - init_ori)*180/math.pi) <= 45:
          HAL.setW(1)
          HAL.setV(-0.5)
        else:
          movement = RIGHT_BACK
        
      elif movement == RIGHT_BACK:
        if ((HAL.getPose3d().yaw - init_ori)*180/math.pi) > 0:
          HAL.setW(-1)
          HAL.setV(-0.5)
          
          if distance_laser(HAL.getBackLaserData()) < 0.8:
            movement = FRONT
        else:
          status = PARKED
          HAL.setV(0)
          HAL.setW(0)
          
      elif movement == FRONT:
        if distance_laser(HAL.getBackLaserData()) > 3 or distance_laser(HAL.getFrontLaserData()) < 1:
          movement = RIGHT_BACK
        else:
          HAL.setV(0.5)
          HAL.setW(-1)
    elif status == PARKED:
      HAL.setV(0)
      HAL.setW(0)