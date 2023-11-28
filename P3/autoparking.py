from GUI import GUI
from HAL import HAL

# Enter sequential code!

def detect_free_spcace():
  f_laser = HAL.getFrontLaserData().values
  r_laser = HAL.getRightLaserData().values
  b_laser = HAL.getBackLaserData().values
  

while True:
    # Enter iterative code!
    front_laser = HAL.getFrontLaserData()
    right_laser = HAL.getRightLaserData()
    back_laser = HAL.getBackLaserData()
    HAL.setV(0.5)
    detect_free_spcace()