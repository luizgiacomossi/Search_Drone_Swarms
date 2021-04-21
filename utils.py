from constants import SCREEN_WIDTH, SCREEN_HEIGHT, PIX2M, M2PIX,SIZE_DRONE
import pygame as pg
from math import atan2, pi
import random
import copy 
import numpy as np
vec = pg.math.Vector2 

def random_color():
    """"
        Picks a random color R,G or B

        :return: color picked
        :rtype : tuple
    """

    rgbl=[255,0,0]
    random.shuffle(rgbl)
    return tuple(rgbl)

def limit(v2, max):
    """
        Limits magnitude of vector2

        :param v2: Vector2 to be normalized
        :type v2: pygame.Vector2
        :param max: maximum length of vector
        :type max: int
        :return v: returns vector 
        :rtype v: vector2
    """
    v = copy.deepcopy(v2)
    if v.length() > max:
        v.scale_to_length(max)
    return v

def constrain(v2,w,h):
    """
        Constrains movement of drone inside the canvas

        :param v2: Vector2 to be constrained
        :type v2: pygame.Vector2
        :param w: maximum width
        :type w: int
        :param h: maximum height
        :type h: int
        :return v2: returns vector within the limits
        :rtype v2: vector2
    """
    if v2.x > w:
        v2.x = w
    if v2.x < 0:
        v2.x = 0 
    if v2.y > h:
        v2.y = h
    if v2.y < 0:
        v2.y = 0
    return v2

class Aircraft(pg.sprite.Sprite):
    """
        Represents a simple visual animated drone 
        Can load sprites, rotate and update animation
    """
    def __init__(self):
        pg.sprite.Sprite.__init__(self)
        self.sprites = []

        for i in range(0,4):
            self.sprites.append(pg.image.load(f'Drone5/sprite_{i}.png'))

        self.atual = 0
        # inherited from the pygame sprite class it is the first element of the drone
        self.image = self.sprites[self.atual]
        # scales down drone sprites to (70,70)
        self.image = pg.transform.scale(self.image,(70,70))
        # rect is inherited from Sprite
        # defines the sprite's position on the screen
        # take the image size
        self.rect = self.image.get_rect()
        
        # pega o canto superior esquerdo, posição qualquer
        #self.rect.topleft = 100,100

    def update(self, position, angle):
        
        # animation update speed is controle by this parameter
        self.atual += 1
        if self.atual >= len(self.sprites):
            self.atual = 0

        self.image = self.sprites[round(self.atual)]
    
        # Rotates image -> angle should be in degrees
        # rotozoom(Surface, angle, scale) -> Surface
        self.image = pg.transform.rotozoom(self.image, -angle*180/pi - 90, SIZE_DRONE* PIX2M)
        self.rect = self.image.get_rect()
        # positions center of rect in acual drone position
        self.rect.center = position.x,position.y

class FlowField():
    def __init__(self, resolution):
        
        self.cols =int(SCREEN_WIDTH/resolution)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT/resolution)  # Rows of the grid
        self.resolution = resolution # Resolution of grid relative to window width and height in pixels

        self.field = [[vec(random.uniform(0,1),random.uniform(0,1)) for col in range(self.cols)] for row in range(self.rows)] # create matrix 
        
    def draw(self, screen):

        blockSize = self.resolution #Set the size of the grid block
        print(self.cols,self.rows)
        for x in range(0, SCREEN_WIDTH, blockSize):
            for y in range(0, SCREEN_HEIGHT, blockSize):
                rect = pg.Rect(x, y, blockSize, blockSize)
                pg.draw.rect(screen, (200,200,200), rect, 1)



