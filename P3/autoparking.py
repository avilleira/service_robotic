from GUI import GUI
from HAL import HAL
# Enter sequential code!

def detect_free_spcace():
  f_laser = HAL.getFrontLaserData()
  r_laser = HAL.getRightLaserData()
  b_laser = HAL.getBackLaserData()

while True:
    # Enter iterative code!
    front_laser = HAL.getFrontLaserData()
    right_laser = HAL.getRightLaserData()
    back_laser = HAL.getBackLaserData()
    print(right_laser.minAngle)
    HAL.setV(0.5)