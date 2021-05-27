import random
import pygame as pg
from utils import Tree

vec2 = pg.math.Vector2
random.seed(3)

class Obstacles(object):
    def __init__(self, num_of_obstacles, map_size):
        super().__init__()
        self.num_of_obstacles = num_of_obstacles
        self.map_size = map_size
        self.obst = []
        
        # Variables to draw tree using Sprites
        self.tree = Tree() 
        self.all_sprites = pg.sprite.Group()
        self.all_sprites.add(self.tree)
        
    def generate_obstacles(self):
        self.obst = []
        
        for _ in range(self.num_of_obstacles):
            self.obst.append(vec2(random.uniform(0,self.map_size[0]),
                                  random.uniform(100,self.map_size[1]))) 
                                  
    def get_coordenates(self):
        return self.obst

    def draw(self):
        self.all_sprites.draw(self.window)
        self.all_sprites.update(self.location,self.rotation)