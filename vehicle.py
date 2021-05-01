import pygame as pg
from utils import Aircraft, random_color, limit, constrain
from constants import *
from math import cos, sin, atan2, pi
import random
import copy 

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
        self.location = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT)) # Random position in screen
        self.velocity = vec2(0.1,0) # Inicial speed
        self.target = vec2(x,y)
        self.acceleration = vec2(0,0)
        self.radius = SIZE_DRONE # Drone Size
        self.desired = vec2()

        self.memory_location = [] # To draw track
        self.rotation = atan2(self.location.y,self.location.x) # inicital rotation

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
    
    def update(self):
        """
            Standart Euler integration
            Updates bahavior tree
        """
        # updates behavior in machine state
        self.behavior.update(self)
        # Updates velocity at every step and limits it to max_speed
        self.velocity += self.acceleration * 1 
        self.velocity = limit(self.velocity, self.max_speed) 
        # updates position
        self.location += self.velocity 
        # Prevents it from crazy spinning due to very low noise speeds
        if self.velocity.length() > 0.5:
            self.rotation = atan2(self.velocity.y,self.velocity.x)
        # Constrains position to limits of screen 
        self.location = constrain(self.location,SCREEN_WIDTH,SCREEN_HEIGHT)
        self.acceleration *= 0

        # Memory of positions to draw Track
        self.memory_location.append((self.location.x,self.location.y))
        # size of track 
        if len(self.memory_location) > SIZE_TRACK:
            self.memory_location.pop(0)

    def applyForce(self, force):
        """
            Applies vetor force to vehicle 
            Newton's second law -> F=m.a
            You can divide by mass
        """
        self.acceleration += force/MASS 

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

    def collision_avoidance(self, all_positions, index):
        """
         This method checks if the drone is colliding with another 
         drone during simulation it receives all the positions from all drones
        """
        # gets all positions of simultaneos drones
        aux = 0 
        soma = vec2(0,0)
        count = 0 # counts the number of drones that are close
        for p in all_positions:
        # compares current position to all the drones
        # aux != index -> avoids the auto-collision check
            d = (self.location - p.location).length()
            separation_factor = 2.2
            if ( (d > 0) and (d < AVOID_DISTANCE*separation_factor) and (aux != index) ) :
                #self.velocity *= - 20 # arbitrary factor that defines how strong is the impact
                diff = (self.location - p.location).normalize()
                diff = diff/d # proporcional to the distance. The closer the stronger needs to be
                soma += diff
                count += 1 # p drone is close 
                #self.applyForce(diff - self.velocity)
                #return 1
            aux+=1
        if count > 0:
            soma = soma / count
            soma = soma.normalize()
            soma *= self.max_speed
            steer = soma - self.velocity
            steer = limit(steer,self.max_force)
            self.applyForce(steer)
            return 1
        else:
            return 0
            
    def draw(self, window):

        """
            Defines shape of vehicle and draw it to screen
        """

        # draws track
  
        # Drawing drone's outer circle as a hitbox?
        if self.debug == True:
            pg.draw.circle(self.window, (100, 100, 100), self.location, AVOID_DISTANCE, 1)
            if len(self.memory_location) >= 2:
                pg.draw.lines(self.window, self.color_target, False, self.memory_location, 1)

            pg.draw.line(self.window, (100, 100, 100), self.location, self.location+self.desired , 1)


        # usar sprite para desenhar drone
        self.all_sprites.draw(self.window)
        self.all_sprites.update(self.location,self.rotation)

 #---- these methods were not used in this project
    def check_collision(self, all_positions, index):
        """
            Not working yet
        """
        aux = 0 
        for p in all_positions:
            d = (self.location - p.location).length()
            factor_distance = 2
            if ( d < AVOID_DISTANCE*factor_distance )  and (aux != index):
                #a = self.velocity.lerp(vec2(-0.2,-0.2), d/(AVOID_DISTANCE))
                #f = (self.velocity.lerp(vec2(0.1,0.1), d/(AVOID_DISTANCE)) - self.velocity )/ SAMPLE_TIME
                self.velocity *= d/(AVOID_DISTANCE*factor_distance)
                #self.applyForce(f*0.01)
                print(f'Alerta de colisão drone {index} com drone {aux}')
                break
            aux +=1

    def bouncing(self):
        """
            Bouncing Behavior
            NOT USED
        """

        if self.location.x + self.radius > SCREEN_WIDTH or self.location.x - self.radius< 0:
            self.velocity.x *= -1
        if self.location.y + self.radius> SCREEN_HEIGHT or self.location.y- self.radius < 0:
            self.velocity.y *= -1

        self.location += self.velocity

    # Deleting (Calling destructor)
    def __del__(self):
        print('Drone Deleted')