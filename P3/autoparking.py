from GUI import GUI
from HAL import HAL

# Enter sequential code!

# MACROS

SEARCHING = 0
ALIGNING = 1
PARKING = 2

front_right_laser = 60


def detect_free_spcace():
  f_laser = HAL.getFrontLaserData().values
  r_laser = HAL.getRightLaserData().values
  b_laser = HAL.getBackLaserData().values
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
      if r_laser[angle] < HAL.getRightLaserData().minRange:
        free_space = True
      else:
        free_space = False
        return free_space
  return True
  
  

status = SEARCHING

while True:
    # Enter iterative code!
    if status == SEARCHING:
      if detect_free_spcace() == True:
        HAL.setV(0)
        print("HUECO ENCONTRADO")
        status = PARKING
      else:
        HAL.setV(1)
    
    elif status == ALIGNING:
      line_up_car()
      