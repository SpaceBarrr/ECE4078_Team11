
import pygame
import cv2
import numpy as np
import sys

(width, height) = (1100, 660)
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('Test map')

map_background_rect = pygame.Rect(700, 0, 400, 660)
background_colour = (45,45,45)
rect_colour = (128,128,128)
screen.fill(background_colour)
pygame.draw.rect(screen,rect_colour,map_background_rect)



second_image = pygame.image.load('map_image.png')
second_image = pygame.transform.scale(second_image, (400, 400))
screen.blit(second_image, (700, 0))
origin_dot = pygame.Rect(904,201,4,4)
origin_colour = (165,42,42)
pygame.draw.rect(screen,origin_colour,origin_dot)

pygame.display.flip()

running = True
while running:
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False
      

