from GUI import GUI
from HAL import HAL
import cv2
import time


# ----------- GLOBAL VARIABLES ------------------
UTM_INIT = [430492, 4459162] # UTM: East and North
UTM_SURV_POSE = [430532, 4459132]

# STATES
TAKE_OFF = 0
PATROLLING = 1
LAND = 2

#DATA
HEIGHT = 2 #m
SURVIVORS = 6
TIME_LIMIT = 600 # 10 min
SQUARE_AREA = 17 #m
TURN_LENGTH = 1

ROTATIONS = [45, 90, 135, 180, 225, 270, 315, 360]
AREA_ERR = 2.5 # Stablize a detection umbral to identify new faces

# DIRECTIONS
SOUTH = 1
EAST = 2
WEST = 3

# ------------------ FUNCTIONS ------------------

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

def return_home():
  HAL.set_cmd_pos(0, 0, 2, 0)
  current_pos = HAL.get_position()
  
  while (round(current_pos[0]) != 0) and (round(current_pos[1]) != 0):
    current_pos = HAL.get_position()
    
    GUI.showLeftImage(HAL.get_ventral_image())
    GUI.showImage(HAL.get_frontal_image())
  
  HAL.land()
  
  state = LAND
  

# Detect if the new face has been seen before
def is_new_person(person_pos):
    new_person = True
    for area in area_survivors:
      if (person_pos[0] >= area[0] and person_pos[0] <= area[1] and person_pos[1] >= area[2] and person_pos[1] <= area[3]):
        new_person = False
    return new_person
    
# Detect new survivors
def face_detection (img):
  rows, cols = img.shape[0], img.shape[1]
  
  # In order to detect a face:
  for angle in ROTATIONS:
    # Rotation matrix around center point
    rotation_matrix = cv2.getRotationMatrix2D((cols // 2, rows // 2), angle, 1)
    rotated_img = cv2.warpAffine(img, rotation_matrix, (cols, rows))
    gray_img = cv2.cvtColor(rotated_img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray_img, 1.1, 5)
    
    for (x, y, w, h) in faces:
      person_x, person_y = HAL.get_position()[0], HAL.get_position()[1]
      if is_new_person (HAL.get_position()):
        # Show founded person image:
        cv2.rectangle(rotated_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        GUI.showImage(rotated_img)
        # Save survivor data:
        area_survivors.append([HAL.get_position()[0] - 3, HAL.get_position()[0] + 3, HAL.get_position()[1] - 3, HAL.get_position()[1] + 3])
        survivors_pos.append([UTM_INIT + HAL.get_position()[0], UTM_INIT + HAL.get_position()[1]])
        print(f"DETECTED SURVIVOR: POSITION {HAL.get_position()[0]},  {HAL.get_position()[1]}")
        return True
        
  return False
  
# Enter sequential code!

state = TAKE_OFF
direction = WEST
prev_dir = 0
posy = 0
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
survivors_detected = 0

survivors_pos = []
# In order to avoid another person is detected at the same time
area_survivors = []

#Init time MAX 10 min
init_time = time.time()
while True:
    # Enter iterative code
    # Check if the time is finished:
    if time.time() - init_time >= TIME_LIMIT:
      print("WARNING: DRONE ALMOST OUT OF POWER. RETURNING TO THE BASE")
      return_home()
    if survivors_detected >= SURVIVORS:
      print("ALL SURVIVORS FOUNDED IN UTM COORDINATES")
      for p in survivors_pos:
        print(f"- UTM: {p[0] + UTM_INIT}, {p[1] + UTM_INIT}")
      return_home()
    # Taking off state
    if state == TAKE_OFF:
      HAL.takeoff(HEIGHT)
      mv_survivors_init_pose()
      init_square_point = HAL.get_position()
      state = PATROLLING
      
    elif state == PATROLLING:
      current_pos = HAL.get_position()
      speedx = 0
      speedy = 0
      if face_detection(HAL.get_ventral_image()) == True:
        survivors_detected += 1
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
        
        if current_pos[0] <= init_square_point[0] - SQUARE_AREA:
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