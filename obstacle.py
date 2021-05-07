import random
from pygame import math

vec2 = math.Vector2

class Obstacles(object):
    def __init__(self, num_of_obstacles, map_size):
        super().__init__()
        self.num_of_obstacles = num_of_obstacles
        self.map_size = map_size
        self.obst = []

    def generate_obstacles(self):
        for _ in range(self.num_of_obstacles):
            self.obst.append(vec2(random.uniform(0,self.map_size[0]),
                            random.uniform(0,self.map_size[1]))) 

    def get_coordenates(self):
        return self.obst
