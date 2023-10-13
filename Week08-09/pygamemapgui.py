
import pygame
import cv2
import numpy as np
import sys
import time

background_colour = (45,45,45)
rect_colour = (128,128,128)
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)


PIBOT_WIDTH = (155/1.5)*0.1
PIBOT_HEIGHT = PIBOT_WIDTH

def gui_xcoord(in_x):
  scalefactor_x = 155/1.5
  x = in_x * scalefactor_x + 906 - PIBOT_WIDTH/2
  return x

def gui_ycoord(in_y):
  scalefactor_y = -155/1.5
  y = in_y * scalefactor_y + 203 - PIBOT_HEIGHT/2
  return y

def initialise_gui_map(map_image,pibot):
    map_image = pygame.transform.scale(map_image, (400, 400))
    canvas.blit(map_image, (700, 0))

    pibot = pygame.transform.scale(pibot, (PIBOT_WIDTH, PIBOT_HEIGHT))
    pibot_x, pibot_y = gui_xcoord(0), gui_ycoord(0)
    canvas.blit(pibot, (pibot_x, pibot_y))

    pygame.display.flip()
    return map_image, pibot

def update_gui_map(robotpose_x, robotpose_y, map_image, pibot): # update this to take in waypoints as well?
  # Update robot's position

  # TESTING
    # pibot_x = robotpose_x
    # pibot_y = robotpose_y

  #use this if inputting coordinates in metres/from robot pose
    pibot_x = gui_xcoord(robotpose_x) # <-- place x coord inside from robot pose 
    pibot_y = gui_ycoord(robotpose_y) # <-- place y coord inside from robot pose 
  

    canvas.blit(map_image, (700, 0))
    canvas.blit(pibot, (pibot_x, pibot_y))
    time.sleep(0.01)
    pygame.display.flip()

# ===============================================================
(width, height) = (1100, 660)
canvas = pygame.display.set_mode((width, height))
pygame.display.set_caption('Test map')

map_background_rect = pygame.Rect(700, 0, 400, 660)
canvas.fill(background_colour)
pygame.draw.rect(canvas,rect_colour,map_background_rect)



map_image = pygame.image.load('map_imagev2.png')
pibot = pygame.image.load('guipngs/pibot_top.png')
map_image, pibot = initialise_gui_map(map_image,pibot)
# =======

# TESTING
# j = 0
# pibot_x = gui_xcoord(0)
# pibot_y = gui_ycoord(0)
# xcoord = 0
# ycoord = 0
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    # TESTING
    # if j <= 100:
    #         pibot_x = pibot_x+1 
    #         pibot_y = pibot_y+1
    #         j = j + 1
    update_gui_map(xcoord, ycoord, map_image, pibot) 
    

