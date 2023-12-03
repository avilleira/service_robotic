from GUI import GUI
from HAL import HAL

# Enter sequential code!

def detect_free_spcace():
  f_laser = HAL.getFrontLaserData().values
  r_laser = HAL.getRightLaserData().values
  b_laser = HAL.getBackLaserData().values
  
  free_space = False

  for angle in range(len(r_laser)):
    if angle >= 40 and angle <= 125:
      if r_laser[angle] > HAL.getRightLaserData().maxRange:
        free_space = True
      else:
        free_space = False
        return free_space
  
  return free_space

while True:
    # Enter iterative code!

    if detect_free_spcace() == True:
      HAL.setV(-0.5)
      print("HUECO ENCONTRADO")
    else:
      HAL.setV(0.5)