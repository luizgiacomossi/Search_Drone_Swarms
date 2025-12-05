import random
import pygame as pg
from utils import Tree
from constants import *

vec2 = pg.math.Vector2


class Obstacles(object):
    def __init__(self, num_of_obstacles, map_size):
        super().__init__()
        self.num_of_obstacles = num_of_obstacles
        self.map_size = map_size
        self.obst = []
        # Use a local Random instance to avoid affecting global state
        self.rng = random.Random(0)
        self.times_generated = 0
        # Variables to draw tree using Sprites
        self.tree = Tree() 
        self.all_sprites = pg.sprite.Group()
        self.all_sprites.add(self.tree)
        
    def generate_obstacles(self):
        self.obst = []
        self.times_generated += 1
        # Seed the local generator, not the global one
        self.rng.seed(self.times_generated+10 )
        valid = False
        for _ in range(self.num_of_obstacles):
            coord = vec2(self.rng.uniform(200,self.map_size[0] + AVOID_OBSTACLES),
                        self.rng.uniform(RADIUS_TARGET,self.map_size[1])- AVOID_OBSTACLES)

            self.obst.append(coord) 
                                  
    def get_coordenates(self):
        return self.obst

    def reset_seed(self):
        self.rng.seed(0)
        self.times_generated = 0
        

    def draw(self):
        self.all_sprites.draw(self.window)
        self.all_sprites.update(self.location,self.rotation)