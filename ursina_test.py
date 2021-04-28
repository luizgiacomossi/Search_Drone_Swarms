from ursina import *
from utils import Aircraft, random_color, constrain3d
from constants import *
from math import cos, sin, atan2, pi
import random
import copy 

app = Ursina()

r = 8
for i in range(1, r):
    t = i/r
    s = 4*i
    print(s)
    grid = Entity(model=Grid(s,s), scale=s, color=color.color(0,0,.8,lerp(.8,0,t)), rotation_x=90, y=i/1000)
    subgrid = duplicate(grid)
    subgrid.model = Grid(s*4, s*4)
    subgrid.color = color.color(0,0,.4,lerp(.8,0,t))
    EditorCamera()

#cub = Entity(model = 'cube', scale = 1, position = (.1,0.5,.1), texture = 'brick')

class Vehicle(Entity):
    def __init__(self):
        super().__init__()
        self.max_speed = FORWARD_SPEED
        self.max_force = SEEK_FORCE
        self.angular_speed = ANGULAR_SPEED
        self.location = Vec3(random.uniform(-10,10),0.1,random.uniform(-10,10)) # Random position in screen
        self.velocity = Vec3(0.1,0,0 ) # Inicial speed
        self.target = Vec3(10,10,10)
        self.acceleration = Vec3(0,0,0) # gravity
        self.radius = SIZE_DRONE # Drone Size
        self.desired = Vec3()
        self.drone = Entity(model = 'Drone_Costum', scale = 0.1,color= color.black, position = self.location , texture = 'brick')

    def update(self):
        self.velocity += self.acceleration * time.dt
        print(self.acceleration )
         #self.velocity = limit3d(self.velocity, self.max_speed) 
    #         #updates position
        self.location += self.velocity 
    #         # Prevents it from crazy spinning due to very low noise speeds
         #if self.velocity.length() > 0.5:
             #self.rotation = atan2(self.velocity.y,self.velocity.x)
    #         # Constrains position to limits of screen 
        self.location = constrain3d(self.location,10,10,10)
        self.drone.position = self.location 
        self.acceleration *=0

    def applyForce(self, force):
        """
            Applies vetor force to vehicle 
            Newton's second law -> F=m.a
            You can divide by mass
        """
        self.acceleration += force/MASS 


    def input(self,key):
        if key == 'd':
            self.applyForce(Vec3(0.5,0,0))
        if key == 'a':
            self.applyForce(Vec3(-0.5,0,0))
        if key == 'w':
            self.applyForce(Vec3(0,1,0))
        if key == 's':
            self.applyForce(Vec3(0,-1.5,0))
        if key == 'q':
            self.applyForce(Vec3(0,0,0.5))
        if key == 'e':
            self.applyForce(Vec3(0,0,-0.5))
        


c = Vehicle()




app.run()