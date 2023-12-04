from GUI import GUI
from HAL import HAL
import math

# Enter sequential code!

MINDISTANCE = 5
# MACROS STATUS

SEARCHING = 0
ALIGNING = 1
PARKING = 2

# MOVEMENTS
LEFT_BACK = 0
BACK = 1
FRONT = 2
RIGHT_BACK = 3

front_right_laser = 60


def detect_free_spcace():
  f_laser = HAL.getFrontLaserData().values
  r_laser = HAL.getRightLaserData().values

  free_space = False

  for angle in range(len(r_laser)):
    if angle >= front_right_laser and angle <= 140:
      if r_laser[angle] > HAL.getRightLaserData().maxRange:
        free_space = True
      else:
        free_space = False
        return free_space
  
  return free_space
  
  
def line_up_car():
  r_laser = HAL.getRightLaserData().values
  for angle in range(len(r_laser)):
    if angle >= 50 and angle <= 130:
      if r_laser[angle] < MINDISTANCE:
        free_space = True
      else:
        free_space = False
        return free_space
  return True
  
def distance_behind():
  b_laser = HAL.getBackLaserData().values
  
  min_val = 100
  
  for angle_value in b_laser:
    if angle_value < min_val and angle_value > HAL.getRightLaserData().minRange:
      min_val = angle_value
  
  return min_val
  
status = SEARCHING
movement = LEFT_BACK
left_back = False

init_ori = HAL.getPose3d().yaw

while True:

    # Enter iterative code!
    if status == SEARCHING:
      if detect_free_spcace() == True:
        HAL.setV(0)
        print("HUECO ENCONTRADO")
        status = ALIGNING
      else:
        HAL.setV(1)
    
    elif status == ALIGNING:
      
      if line_up_car() == True:
        HAL.setV(0)
        status = PARKING
        print("ALINEADO")
        # We keep current orientation
        init_ori = HAL.getPose3d().yaw
      else:
        HAL.setV(1)
        
    elif status == PARKING:
      if movement == LEFT_BACK:
        if ((HAL.getPose3d().yaw - init_ori)*180/math.pi) <= 50:
          HAL.setW(0.5)
          HAL.setV(-0.5)
        else:
          movement = RIGHT_BACK
        
      elif movement == RIGHT_BACK:
        if ((HAL.getPose3d().yaw() - init_ori)*180/math.pi) > 0:
          HAL.setW(-1)
          HAL.setV(-0.5)
          
          if distance_behind() < 3:
            if round(init_ori - HAL.getPose3d.yaw()) == 0: 
              HAL.setV(1)
        else:
          movement = RIGHT_BACK
        
    