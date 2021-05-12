import pygame
from constants import *
from vehicle import Vehicle, VehiclePF
from state_machine import FiniteStateMachine, SeekState, StayAtState, OvalState, Eight2State, ScanState

vec2 = pygame.math.Vector2
##=========================

class ScreenSimulation(object):

    def __init__(self):
        pygame.init()
        self.font20 = pygame.font.SysFont(None, 20)
        self.font24 = pygame.font.SysFont(None, 24)
        self.size = SCREEN_WIDTH, SCREEN_HEIGHT 
        self.clock = pygame.time.Clock()
        self.screen = pygame.display.set_mode(self.size)


class Simulation(object):
    
    def __init__(self, screenSimulation):
        self.screenSimulation = screenSimulation

        # state machines for each vehicle
        self.behaviors =[] 
        
        # Current simulations 
        self.swarm = []

    def create_swarm_uav(self, num_swarm):
        # Create N simultaneous Drones
        for d in range(0, num_swarm):
            self.behaviors.append( FiniteStateMachine( SeekState() ) ) # Inicial state
            #using Old vehicle: steering behavior
            #drone = Vehicle(SCREEN_WIDTH*d/num_swarm, 10, self.behaviors[-1], self.screenSimulation.screen)
            #using potential fields
            drone = VehiclePF(SCREEN_WIDTH*d/num_swarm, 10, self.behaviors[-1], self.screenSimulation.screen)

            #drone = Vehicle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, behaviors[-1], screen)
            #drone.set_target(vec2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2))
            self.swarm.append(drone)

    def add_new_uav(self):
        self.behaviors.append( FiniteStateMachine( SeekState() ) )
        drone = VehiclePF(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, self.behaviors[-1], self.screenSimulation.screen)
        drone.set_target(vec2(pygame.mouse.get_pos()[0],pygame.mouse.get_pos()[1]))
        self.append_uav(drone)
    
    def append_uav(self, drone):
        self.swarm.append(drone)

    def set_target(self, target):
        for _ in self.swarm:
            _.set_target(target)

    def run_simulation(self, list_obst):
        index = 0 # index is used to track current drone in the simulation list
        for _ in self.swarm:
            # checks if drones colided with eachother

            ## collision avoindance is not implemented yet
            _.collision_avoidance(self.swarm,index)
            _.check_collision(self.swarm,list_obst,index) 
            _.update()
            _.draw(self.screenSimulation.screen) 
            # index to keep track of  drone in the list
            index += 1
            # writes drone id
            img = self.screenSimulation.font20.render(f'Drone {index}', True, BLUE)
            self.screenSimulation.screen.blit(img, _.get_position()+(0,20))
            # writes drone current behavior
            img = self.screenSimulation.font20.render(_.behavior.get_current_state(), True, BLUE)
            self.screenSimulation.screen.blit(img, _.get_position()+(0,30))
            # writes drone current position in column and row
            p = _.get_position()
            col =  int(p.x/RESOLUTION) + 1
            row = int(p.y/RESOLUTION) + 1
            img = self.screenSimulation.font20.render(f'Pos:{col},{row}', True, BLUE)
            self.screenSimulation.screen.blit(img, _.get_position()+(0,40))