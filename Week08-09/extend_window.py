
import pygame
import cv2
import numpy as np
import sys
from Obstacle import *

(width, height) = (1100, 660)
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('Test map')

map_background_rect = pygame.Rect(700, 0, 400, 660)
background_colour = (45,45,45)
rect_colour = (128,128,128)
screen.fill(background_colour)
pygame.draw.rect(screen,rect_colour,map_background_rect)

white = (255, 255, 255)
black = (0, 0, 0)


obstacle_list = [Circle(0.7, 0, 0.3),Circle(-1, -1.2, 0.3),Circle(-0.5, 0, 0.3),Circle(0, 1, 0.3)]



second_image = pygame.image.load('map_image.png')
second_image = pygame.transform.scale(second_image, (400, 400))
screen.blit(second_image, (700, 0))
origin_dot = pygame.Rect(904,201,4,4)
origin_colour = (165,42,42)
pygame.draw.rect(screen,origin_colour,origin_dot)

pygame.display.flip()

unscaled_coords = [[0,0],[0.5,0.5],[1,1],[1.2,1],[1.2,0],[1,-1],[-0.5,-1],[-1.1,-0.5]]
scalefactor_x = 155/1.5
scalefactor_y = -155/1.5
coordinates = [[scalefactor_x * x + 906, scalefactor_y * y + 203] for x, y in unscaled_coords]
white = (255, 255, 255)
red = (255, 0, 0)



for i in range(len(coordinates) - 1):
        pygame.draw.line(screen, white, coordinates[i], coordinates[i + 1], 2)

for circle in obstacle_list:
  pygame.draw.circle(screen, red, (int(circle.center[0] * scalefactor_x+906), int(circle.center[1] * scalefactor_y+203)), circle.radius*scalefactor_x)
pygame.display.flip()

running = True
while running:
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False
      

