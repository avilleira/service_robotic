from GUI import GUI
from HAL import HAL


# ----------- GLOBAL VARIABLES ------------------
UTM_INIT = [430492, 4459162] # UTM: East and North
UTM_SURV_POSE = [430532, 4459132]
TAKE_OFF = 0
PATROLLING = 1
HEIGHT = 2 #m


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
  
  
# Enter sequential code!

state = TAKE_OFF

while True:
    # Enter iterative code
    # Taking off state
    if state == TAKE_OFF:
      HAL.takeoff(HEIGHT)
      mv_survivors_init_pose()
      init_point = HAL.get_position()
      state = PATROLLING
      
    elif state == PATROLLING:
      HAL.set_cmd_mix(0, 1, 2, 0)
      current_pos = HAL.get_position()
      if current_pos[1] >= init_point[1] + 20:
        print("HE LLEGAO")
      
    GUI.showLeftImage(HAL.get_ventral_image())
    GUI.showImage(HAL.get_frontal_image())