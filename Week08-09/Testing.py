import sys, pygame
from pygame.locals import*

width = 300
height = 300
screen_color = (49, 150, 100)
line_color = (255, 0, 0)

def main():
    image = pygame.image.load("map_image.png")
    #screen=pygame.display.set_mode((width,height))
    #screen.fill(screen_color)
    pygame.draw.line(image, line_color, (-14, 0.4), (150, 94))
    pygame.draw.line(image, line_color, (150, 94), (270, 94))
    pygame.display.flip()

    while True:
        for events in pygame.event.get():
            if events.type == QUIT:
                sys.exit(0)

main()