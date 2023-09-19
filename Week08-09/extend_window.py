
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

second_image = pygame.image.load('Week08-09/map_image.png')
second_image = pygame.transform.scale(second_image, (400, 400))
screen.blit(second_image, (700, 0))


pygame.display.flip()

running = True
while running:
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False
      

