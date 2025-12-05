import pygame as pg
from utils import Aircraft, random_color, limit, constrain, bivariateFunction, derivativeBivariate, normalFunction
from constants import *
from math import cos, sin, atan2, pi, inf
import random
import copy 
import numpy as np
from typing import List, Optional, Tuple, Union
from quadtree import Quadtree, Rect


vec2 = pg.math.Vector2

class Vehicle(object):

    def __init__(self, x,y, behavior, window):
        """
            idealized vehicle representing a drone

            :param x and y: represents inicial target 
            :param behavior: State Machine 
            :param window: pygame screen were it will be draw
        """

        self.debug = False #  debug lines is Off

        # Variables used to move drone 
        self.location = vec2(x,y) # Random position in screen
        self.velocity = vec2(0.1,0) # Inicial speed
        self.mission_target = vec2(x,y)
        self.target = vec2(x,y)
        self.acceleration = vec2(0,0)
        self.radius = SIZE_DRONE # Drone Size
        self.desired = vec2()
        # closest drone
        self.closest_drone = None
        self.index_closest_drone = None

        self.memory_location = [] # To draw track
        self.rotation = atan2(self.location.y, self.location.x) # inicital rotation

        # Arbitrary values
        self.max_speed = FORWARD_SPEED
        self.max_force = SEEK_FORCE
        self.angular_speed = ANGULAR_SPEED

        # Picks a random color for target, is used to differentiate visually during simulation
        self.color_target = random_color() 

        # Variables related to State Machine
        self.behavior = behavior
        self.window = window # tela em que esta acontecendo a simulaçao
        self.theta = 0 # variavel para o eight somada no seek_around
        self.count = 0

        # Variables to draw drone using Sprites
        self.drone = Aircraft() 
        self.all_sprites = pg.sprite.Group()
        self.all_sprites.add(self.drone)

        # variables to search in grid
        self.position_in_grid = (0,0)
        self.grid_map = None
        self.found = False

    def reached_goal(self, target):
        return target and (target - self.location).length() <= RADIUS_TARGET 
    
    def applyForce(self, force):
        self.acceleration += force/MASS # Newton's second law
        #f = limit(force , self.max_force)
        #self.acceleration += force/MASS

    def update(self):
        # adds drag force
        self.apply_drag()
        # updates behavior in machine state
        self.behavior.update(self)
        # Updates velocity at every step and limits it to max_speed
        self.velocity += self.acceleration * 1
        self.velocity = limit(self.velocity, self.max_speed)
        # updates position
        self.location += self.velocity
        # Prevents it from crazy spinning due to very low noise speeds
        if self.velocity.length() > 0.8:
            self.rotation = atan2(self.velocity.y,self.velocity.x)
        # Constrains position to limits of screen
        self.location = constrain(self.location,WORLD_WIDTH,WORLD_HEIGHT)
        self.acceleration *= 0

        # Memory of positions to draw Track
        self.memory_location.append((self.location.x,self.location.y))
        # size of track
        if len(self.memory_location) > SIZE_TRACK:
            self.memory_location.pop(0)

    def apply_drag(self):
        """
        Applies aerodynamic drag force to the drone.
        """
        drag_coefficient = 0.5  # Adjust this value
        area = 0.1  # Approximate frontal area (adjust this)
        air_density = 1.225  # kg/m^3 (sea level)

        speed_squared = self.velocity.length_squared() # Avoid square root

        drag_magnitude = 0.5 * air_density * area * drag_coefficient * speed_squared
        drag_direction = -self.velocity.normalize()

        drag_force = drag_direction * drag_magnitude
        self.applyForce(drag_force)

    def seek(self, target):
        """
            Seek Steering force Algorithm
        """
        try:
            self.desired  = (target - self.location).normalize()*self.max_speed
        except: # if you try to normalize a null vector it will catch
            self.desired  = (target - self.location)*self.max_speed
        
        # Calculates steering force
        steer = self.desired  - self.velocity
        # Limit the magnitude of the steering force.
        steer = limit(steer,self.max_force)
        # Applies steering force to drone
        self.applyForce(steer)
        # Draws current target being seeked 
        pg.draw.circle(self.window, self.color_target ,target ,5, 0)
    
    def arrive_new(self, target):
        """
            Arrive using potential fields 
        """
        # Calculates vector desired 
        velocity_attract = vec2(0,0)
        velocity_repulsion= vec2(0,0)

        velocity_attract = derivativeBivariate(.05,.05,target,self.location)

        desired_velocity = (velocity_attract - velocity_repulsion) 
        error = (desired_velocity - self.velocity) / SAMPLE_TIME 
        
        accelerate = limit(error, self.max_force)
        self.applyForce(accelerate)
        # Draws current target as a point 
        pg.draw.circle(self.window, self.color_target ,target ,5, 0)

    def arrive_pv(self, target):
        """
            Arrive using position controler PV
        """
        # Calculates vector desired position
        #kp = 0.0024
        #kp = 50
        #kv = 800
        xi=0.7
        wn=10/60
        kv = 2*MASS*xi*wn
        kp = MASS*wn**2
        self.desired = kp * (target - self.location) - kv * self.velocity

        a_desired =  limit(self.desired, self.max_force)

        self.applyForce(a_desired)
        # Draws current target as a point 
        pg.draw.circle(self.window, self.color_target ,target ,5, 0)

    def arrive(self, target):
        """
            Arrive Steering Behavior
        """
        # Calculates vector desired 
        self.desired = (target - self.location)
        # get the distance to the target
        d = self.desired.magnitude() 

        try:
            dist = copy.deepcopy(self.desired.normalize()) # obtem direção
        except: # If the magnitude of desired is zero it cant be normalized
            dist = copy.deepcopy(self.desired)
        
        r = RADIUS_TARGET
        # Modulates the force
        if d < r : # close to target it will reduce velocty till stops
            # interpolation
            dist *= self.max_speed*(1 + 1/r*(d-r))
        else:
            dist *= self.max_speed

        # Steering force
        steer = dist - self.velocity
        #Limit the magnitude of the steering force.
        steer = limit(steer, self.max_force)
        # apply force to the vehicle
        self.applyForce(steer)
        # Simulates Wind - random Noise
        wind = vec2(random.uniform(-0.15,0.15) , random.uniform(-0.15,0.15)  )
        self.applyForce(wind)
        # Draws current target as a point 
        pg.draw.circle(self.window, self.color_target ,target ,5, 0)

    def stay_at(self, center, r = RADIUS_TARGET):
        """
           Drone Behavior - it will orbit a given target (center)
        """
        posToCenter = center - self.location 
        #ok
        if self.debug == True:
            pg.draw.line(self.window,BLACK, self.location ,center,1)

        # se o veiculo se encontra mais longue q o raio de rotaçao
        if posToCenter.length() > r :
            self.seek(center)
            #self.target =copy.deepcopy(center) 
        else: # se ele esta dentro do raio de rotaçao
            # reinicia forças
            centerToPerimeter = posToCenter.normalize()*(-1*r )
            #ok
            pg.draw.line(self.window,(0,0,255),center,center+centerToPerimeter,5 )
            
            posToPerimeter = centerToPerimeter + posToCenter 
            #pg.draw.line(window,(255,0,0),center,center+posToPerimeter,5 )

            print(f'distancia até perimetro {posToPerimeter.length()}')

            # new target is on the radius
                # theta is the angle of the vector center to perimeter
            theta = atan2(centerToPerimeter.y, centerToPerimeter.x)
            theta += self.angular_speed
            new_target = vec2(0,0)

            # new target
            new_target.x += r  * cos(theta)
            new_target.y += r  * sin(theta)
            new_target += center

            if self.debug == True:
                pg.draw.line(self.window,(0,255,0), center,  new_target ,5)# verde é o target
                pg.draw.line(self.window,BLACK, self.location, new_target, 2 )
            
            self.seek(new_target)

    def seek_around(self, center, radius_target = RADIUS_TARGET):
        """
           Drone Behavior - it will orbit a given target (center) with prevision 

           :param center: position of target to  orbite
           :param radius_target: distance till center, default = RADIUS_TARGET from constants
        """
        # Calculating the max speed
        self.angular_speed = FORWARD_SPEED / radius_target

        # future positiom
        hop_ahead = HOP_AHEAD #o quanto se ve a frente
        fut_pos = self.velocity.normalize()*(hop_ahead)
        fut_pos += self.location

        if self.debug == True:
            pg.draw.line(self.window,(0,255,50),self.location,fut_pos,5)
        #print(f'center: {center}')
        posToCenter = center - fut_pos
        # line from drone to center
        if self.debug == True:
            pg.draw.line(self.window,BLACK, self.location ,center,1)

        # se o veiculo se encontra mais longue q o raio de rotaçao
        if posToCenter.length() > radius_target:
            self.seek(center)
            #self.target =copy.deepcopy(center) 
        else: # se ele esta dentro do raio de rotaçao
            # reinicia forças
            centerToPerimeter = posToCenter.normalize()*(-1*radius_target)
            #ok
            if self.debug == True:
                pg.draw.line(self.window,(0,0,255),center,center+centerToPerimeter,5 )
            
            posToPerimeter = centerToPerimeter + posToCenter 
            #pg.draw.line(window,(255,0,0),center,center+posToPerimeter,5 )

            #print(f'distancia até perimetro {posToPerimeter.length()}')

            # new target is on the radius
                # theta is the angle of the vector center to perimeter
            self.theta = atan2(centerToPerimeter.y, centerToPerimeter.x)
            self.theta += self.angular_speed
            new_target = vec2(0,0)

            # new target
            new_target.x += radius_target * cos(self.theta)
            new_target.y += radius_target * sin(self.theta)
            new_target += center
            if self.debug == True:
                pg.draw.line(self.window,(0,255,0), center,  new_target ,5)# verde é o target
                pg.draw.line(self.window,BLACK, self.location, new_target, 2 )
            self.seek(new_target)

    def mission_accomplished(self):
        if self.target :
            return self.location.x == self.target.x and self.location.y == self.target.y
        else:    
            return False

    def get_position(self):
        return self.location

    def set_target(self, target):
        self.target = target
    
    def get_target(self):
        try:
            return self.target
        except: 
            return None

    def set_debug(self):
        """
        Method to view debug lines . Assists the developer.
        """
        self.debug = not self.debug

    def get_debug(self):
        return str(self.debug)

    def align_direction_with_swarm(self, quadtree: Quadtree, index):
        """
         This method avoids collisions with other drones
         During simulation it receives the quadtree containing all drones 
         index: is the current id of drone being checked 
        """
        # Define search range
        separation_factor = 2.2
        search_radius = AVOID_DISTANCE * separation_factor
        range_rect = Rect(self.location.x, self.location.y, search_radius, search_radius)
        
        # Query quadtree for nearby drones
        nearby_drones = []
        quadtree.query(range_rect, nearby_drones)

        soma = vec2(0,0) # sums up all directions of close drones
        count = 0 # counts the number of drones that are close
        
        for p in nearby_drones:
            # compares current position to all the drones
            # p != self -> avoids the auto-collision check
            if p == self:
                continue
                
            d = (self.location - p.location).magnitude()
            
            if (d > 0) and (d < search_radius):
                diff = (self.location - p.location).normalize()
                diff = diff/d # proporcional to the distance. The closer the stronger needs to be
                soma += diff
                count += 1 # p drone is close 
            
        if count > 0:
            media = soma / count
            media = media.normalize()
            media *= self.max_speed
            steer = (media - self.velocity)
            steer = limit(steer,self.max_force)
            #----
            #----
            self.applyForce(steer)
                  
    def draw(self, window):

        """
        Render drone with optional debug visualization.
        
        Args:
            window: Pygame window surface
        """
        # Draw connection to closest drone
        if self.closest_drone:
            pg.draw.line(self.window, self.color_target, self.location, self.closest_drone, 1)

        # Render drone track
        if len(self.memory_location) >= 2:
            pg.draw.lines(self.window, self.color_target, False, self.memory_location, 1)

        # Debug visualization
        if self.debug:
            pg.draw.circle(self.window, (100, 100, 100), self.location, AVOID_DISTANCE, 1)
            v = self.velocity.length()
            pg.draw.line(self.window, self.color_target, self.location, 
                         self.location + self.velocity.normalize() * v * 20, 1)

        # Render sprite
        self.all_sprites.draw(self.window)
        self.all_sprites.update(self.location, self.rotation)


    def collision_avoidance(self, quadtree: Quadtree, pos_obstacles: List[vec2], index: int) -> None:
        """
        Advanced collision avoidance with drones and obstacles.
        
        Args:
            quadtree (Quadtree): Quadtree containing all drones
            pos_obstacles (List[vec2]): Obstacle positions
            index (int): Current drone's index
        """
        # Drone collision avoidance
        closest = +inf
        self.closest_drone = None
        factor_distance = 2
        search_radius = AVOID_DISTANCE * factor_distance
        
        # Query quadtree for nearby drones
        range_rect = Rect(self.location.x, self.location.y, search_radius, search_radius)
        nearby_drones = []
        quadtree.query(range_rect, nearby_drones)

        for p in nearby_drones:
            if p == self:
                continue
                
            d = (self.location - p.location).magnitude()
            
            # Find closest drone
            if 0 < d < closest:
                closest = d
                self.closest_drone = copy.deepcopy(p.location)
            
            # Collision prevention
            if d < search_radius:
                f_repulsion = derivativeBivariate(0.001, 0.001, p.location, self.location) / SAMPLE_TIME
                f_repulsion = limit(f_repulsion, self.max_force * 1.8)
                self.applyForce(-f_repulsion)
                break

        # Obstacle collision avoidance
        factor_repulsion = 0.005
        dist_avoid = AVOID_OBSTACLES + AVOID_DISTANCE

        for obstacle in pos_obstacles:
            d = (self.location - obstacle).length()
            
            if d < dist_avoid:
                f_repulsion = derivativeBivariate(factor_repulsion, factor_repulsion, obstacle, self.location) / SAMPLE_TIME
                f_repulsion = limit(f_repulsion, self.max_force * 1.8)
                self.applyForce(-f_repulsion)
                
                # Emergency collision handling
                if d < RADIUS_OBSTACLES + SIZE_DRONE:
                    self.velocity *= -0.5

    def get_closest_drone(self):
        return self.closest_drone
    
    def set_position_in_grid(self,x,y):
        # x,y in grid cell
        self.position_in_grid = (x,y)

    def get_position_in_grid(self):
        return self.position_in_grid 

    def save_grid(self, grid):
        self.grid_map = grid

    # Deleting (Calling destructor)
    #def __del__(self):
        #print('Drone Deleted')
