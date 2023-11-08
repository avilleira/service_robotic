from GUI import GUI
from HAL import HAL

UTM_INIT = [430492, 4459162] # UTM: East and North
UTM_SURV_POSE = [430532, 4459132]


def mv_survivors_init_pose ():
  # Calculating survivors position using UTM
  east_dest = UTM_SURV_POSE[0] - UTM_INIT[0]
  north_dest = UTM_SURV_POSE[1] - UTM_INIT[1]
  current_pos = HAL.get_position()
  
  # Move drone to the initial pose of the survivors:
  HAL.set_cmd_pos(east_dest, north_dest, 2, 0)
  
  while (round(current_pos[0]) != east_dest) and (round(current_pos[1]) != north_dest):
    current_pos = HAL.get_position()
  
  
# Enter sequential code!
print("INICIANDO PROGRAMA")
HAL.takeoff(2)
print("DESPEGUE")

while True:
    # Enter iterative code
    GUI.showLeftImage(HAL.get_ventral_image())
    GUI.showImage(HAL.get_frontal_image())
    
    mv_survivors_init_pose()