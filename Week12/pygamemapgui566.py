
import pygame
import numpy as np
import time

background_colour = (45,45,45)
rect_colour = (128,128,128)
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
orange = (245, 117, 20)

#old scaling 155/1.5 (for 400x400px map) as in 155px/1.5m
# double area is 400*sqrt(2) =~566x566px map with new scaling 218/1.5
#origin is at 906,203 for 400px, for 566px at 991,288


PIBOT_WIDTH = (218/1.5)*0.1
PIBOT_HEIGHT = PIBOT_WIDTH


def gui_xcoord(in_x):
  scalefactor_x = 218/1.5
  x = in_x * scalefactor_x + 991 - PIBOT_WIDTH/2
  return x

def gui_ycoord(in_y):
  scalefactor_y = -218/1.5
  y = in_y * scalefactor_y + 288 - PIBOT_HEIGHT/2
  return y

def wp_xcoord(in_x):
  scalefactor_x = 218/1.5
  x = in_x * scalefactor_x + 991
  return x

def wp_ycoord(in_y):
  scalefactor_y = -218/1.5
  y = in_y * scalefactor_y + 288
  return y


def initialise_map(canvas, map_image):
    map_image = pygame.transform.scale(map_image, (566, 566))
    canvas.blit(map_image, (700, 0))
    pygame.display.flip()
    return map_image

def update_gui_map(canvas, robotpose_x, robotpose_y, robotpose_theta, map_image, pibot, waypoints): # update this to take in waypoints as well?
  # Update robot's position

  # TESTING ================================================================
  # pibot_x = robotpose_x
  # pibot_y = robotpose_y
  # pibot_angle = robotpose_theta
  # ========================================================================

  #use this if inputting coordinates in metres/from robot pose
  pibot_x = gui_xcoord(robotpose_x) # <-- place x coord inside from robot pose 
  pibot_y = gui_ycoord(robotpose_y) # <-- place y coord inside from robot pose 
  pibot_angle = np.degrees(robotpose_theta) # <-- place theta inside from robot pose in radians
  
  gui_waypoints = [(wp_xcoord(x), wp_ycoord(y)) for x, y in waypoints]
  pibot_line_x = pibot_x+PIBOT_WIDTH/2
  pibot_line_y = pibot_y+PIBOT_HEIGHT/2


  canvas.blit(map_image, (700, 0))
  for wx, wy in gui_waypoints:
        pygame.draw.circle(canvas, red, (wx, wy), 3)
  for i in range(len(gui_waypoints) - 1):
        pygame.draw.line(canvas, red, gui_waypoints[i], gui_waypoints[i+1])
  if gui_waypoints:  # Check if there's at least one waypoint
        pygame.draw.line(canvas, red, (pibot_line_x, pibot_line_y), gui_waypoints[0])
  pibot = pygame.transform.rotate(pibot,pibot_angle)
  canvas.blit(pibot, (pibot_x, pibot_y))
  time.sleep(0.01)
  pygame.display.flip()

# ===============================================================
# background_colour = (45,45,45)
# rect_colour = (128,128,128)
# white = (255, 255, 255)
# black = (0, 0, 0)
# red = (255, 0, 0)
# orange = (245, 117, 20)

# PIBOT_WIDTH = (218/1.5)*0.1
# PIBOT_HEIGHT = PIBOT_WIDTH

# (width, height) = (1100-400+566, 660)
# canvas = pygame.display.set_mode((width, height))
# pygame.display.set_caption('Test map')

# map_background_rect = pygame.Rect(700, 0, 566, 660)
# canvas.fill(background_colour)
# pygame.draw.rect(canvas,rect_colour,map_background_rect)



# map_image = pygame.image.load('map_imagev2.png')
# pibot = pygame.image.load('guipngs/pibot_top.png')
# pibot = pygame.transform.scale(pibot, (PIBOT_WIDTH, PIBOT_HEIGHT))

# map_image = initialise_map(canvas,map_image)
# ========================================================================

# # TESTING ================================================================
# j = 0
# pibot_x = gui_xcoord(0)
# pibot_y = gui_ycoord(0)
# pibot_theta = 0
# waypoints = [[0,0],[0.25,0.25],[0.5,0.5],[0.75,0.75],[1,1]]
# # =======================================================================

# running = True
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#     # TESTING ==================================================================
    # if j == 1:
    #     operate.notification = f"Travelling to fruit: {fruit_to_find}"

#     if j <= 142: 
#       pibot_x = pibot_x+1  #pos x dir
#       pibot_y = pibot_y-1  #pos y dir
#       pibot_theta = pibot_theta+0.0872665 # + 5 deg in rad
#       j = j + 1
#     if j == 143: # treat this as the event of finding fruit and looking for next
#       #operate.notification = f"Found fruit: {fruit_to_find}!"
#       waypoints = [[0,0],[-0.25,-0.25],[-0.75,-0.75],[-1,-1]]
#       time.sleep(1)
#       j = j + 1
#       #operate.notification = f"Travelling to fruit: {fruit_to_find}"
#     if j >143 and j<426:
#       pibot_x = pibot_x-1  #neg x dir
#       pibot_y = pibot_y+1  #neg y dir
#       pibot_theta = pibot_theta+0.0872665 # + 5 deg in rad
#       if j== 200:
#         waypoints = [[0,-0.25],[-0.75,-0.75],[-1,-1]]

#       j = j + 1
# #     # ========================================================================

#     update_gui_map(canvas, pibot_x, pibot_y, pibot_theta, map_image, pibot, waypoints) 
    
    

