import pygame
import sys
from PyQt5.QtWidgets import QApplication
import numpy as np
import matplotlib.pyplot as plt



x_length = 30 # in m
y_length = 20 # in m

def sinus_path():
    x = np.arange(0, 25, 0.01).reshape(-1,1)
    y = np.sin(x).reshape(-1,1) + 5
    path = np.hstack((x, y))
    return path


# def meter_to_pixel(val):



# print(screen.get_size())

pygame.init()


# screen = pygame.display.set_mode((pixel_window_width, pixel_window_height))
screen = pygame.display.set_mode((0, 0), pygame.RESIZABLE)
print(pygame.display.get_surface().get_size())
width, height = pygame.display.get_surface().get_size()
x_length = 30 # in m
scale_to_pixel = width / x_length
# print(ppm, width / height, ppm_y)
print(width, height)
print("scale_to_pixel:", scale_to_pixel)
clock = pygame.time.Clock()
running = True

white = [255, 255, 255]
screen.fill(white)


points = sinus_path() 
print("0:", points[0,1])
points *= scale_to_pixel

# points[:,0] = width/2) - (points[:,0]/2)
print(height - points[0,1], height)
points[:,1] = height - points[:,1]
print("0:", points[0,1], points[0,1]/scale_to_pixel)
pygame.draw.lines(surface=screen, color=(0,0,0), closed=False, points=points, width=1)



image = pygame.image.load("tractor.png")
print("M:", image.get_height(), image.get_width())
tractor_x = 3.0
print("XXXs",scale_to_pixel*tractor_x)
tractor_width = int(scale_to_pixel*tractor_x)
tractor_height = int(image.get_height() * tractor_width/image.get_width() )
print(tractor_width, tractor_height, image.get_height())
image = pygame.transform.scale(image, (tractor_width, tractor_height))



while running:
    screen.blit(image, (0, 0))
    screen.blit(image, (184, 0))
    screen.blit(image, (184*2, 0))
    screen.blit(image, (184*9, 0))
    clock.tick(60)
    for event in pygame.event.get():
        if event == pygame.QUIT:
            running = False

    pygame.display.flip()