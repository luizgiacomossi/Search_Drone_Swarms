import time
import pygame
from constants import *
from vehicle import Vehicle, VehiclePF
from scan import ScanInterface
from state_machine import FiniteStateMachine, SeekState, StayAtState, OvalState, Eight2State, ScanState
from random import uniform
from obstacle import Obstacles

vec2 = pygame.math.Vector2
##=========================
class RateSimulation(object):
    def __init__(self, in_repetitions, in_num_swarm, in_algorithms):
        self.current_repetition = 0
        
        # Inputs of Rate
        self.in_repetitions = in_repetitions * len(in_num_swarm) * len(in_algorithms)
        
        self.in_num_swarm = []
        for n in in_num_swarm:
            self.in_num_swarm = self.in_num_swarm + [n] * int(self.in_repetitions/len(in_num_swarm))
        
        self.in_algorithms = []
        for a in in_algorithms:
            self.in_algorithms = self.in_algorithms + [a] * int(self.in_repetitions/len(in_algorithms))

        # Outputs of Rate
        self.out_time = []
        self.print_simulation()

    def set_out(self, out_time):
        self.out_time.append(out_time)

    def next_simulation(self):
        if self.in_repetitions - 1 == self.current_repetition:
            return False
        else:
            self.current_repetition = self.current_repetition + 1
            self.print_simulation()
            return True

    def print_simulation(self):
        print(f'{self.current_repetition+1} - num_swarm: {self.in_num_swarm[self.current_repetition]}, Algorithm: {self.in_algorithms[self.current_repetition].to_string()}')


class ScreenSimulation(object):

    def __init__(self):
        pygame.init()
        self.font20 = pygame.font.SysFont(None, 20)
        self.font24 = pygame.font.SysFont(None, 24)
        self.size = SCREEN_WIDTH, SCREEN_HEIGHT 
        self.clock = pygame.time.Clock()
        self.screen = pygame.display.set_mode(self.size)


class Simulation(object):
    
    def __init__(self, screenSimulation,rate:RateSimulation, num_obstacles = NUM_OBSTACLES):
        self.target_simulation = None
        self.screenSimulation = screenSimulation
        self.start_watch = 0
        self.stop_watch = 0
        self.rate = rate
        self.time_executing = 0 
        # variables for obstacles
        self.obstacles = Obstacles(num_obstacles, (SCREEN_WIDTH,SCREEN_HEIGHT))
        self.list_obst = []
        self.generate_obstacles()
        # state machines for each vehicle
        self.behaviors =[] 
        
        # Current simulations 
        self.swarm = []

        self.create_swarm_uav(rate.in_num_swarm[0])

    def generate_obstacles(self):
        # Generates obstacles
        self.obstacles.generate_obstacles()
        self.list_obst = self.obstacles.get_coordenates()

    def create_swarm_uav(self, num_swarm):
        # Create N simultaneous Drones
        for d in range(0, num_swarm):
            self.behaviors.append( FiniteStateMachine( SeekState() ) ) # Inicial state
            #using Old vehicle: steering behavior
            drone = Vehicle(SCREEN_WIDTH*d/num_swarm, 10, self.behaviors[-1], self.screenSimulation.screen)
            
            #using potential fields
            #drone = VehiclePF(SCREEN_WIDTH*d/num_swarm, 10, self.behaviors[-1], self.screenSimulation.screen)

            #drone = Vehicle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, behaviors[-1], screen)
            #drone.set_target(vec2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2))

            self.swarm.append(drone)

    def add_new_uav(self):
        self.behaviors.append( FiniteStateMachine( SeekState() ) )
         #using Old vehicle: steering behavior
        drone = Vehicle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, self.behaviors[-1], self.screenSimulation.screen)

        #using potential fields
        #drone = VehiclePF(SCREEN_WIDTH*d/num_swarm, 10, self.behaviors[-1], self.screenSimulation.screen)

        drone.set_target(vec2(pygame.mouse.get_pos()[0],pygame.mouse.get_pos()[1]))
        self.append_uav(drone)
    
    def append_uav(self, drone):
        self.swarm.append(drone)

    def set_target(self, target):
        self.target_simulation = target
        for _ in self.swarm:
            _.set_target(target)

    def run_simulation(self):
        if self.target_simulation:
            pygame.draw.circle(self.screenSimulation.screen, (100, 100, 100), self.target_simulation, RADIUS_TARGET, 2)

        for _ in self.list_obst:
            pygame.draw.circle(self.screenSimulation.screen,(100, 100, 100), _, radius=RADIUS_OBSTACLES)

        if self.start_watch == 0:
            self.start_watch = time.time()

        self.rate.in_algorithms[self.rate.current_repetition].scan(self, self.list_obst)
        
        self.time_executing += SAMPLE_TIME # count time of execution based on the sampling
        print(self.time_executing)

        if self.completed_simualtion() >= 0.8 and self.stop_watch == 0 or self.time_executing > TIME_MAX_SIMULATION:
            self.stop_watch = time.time()
            
            if self.rate and self.rate.next_simulation():
                self.rest_simulation()
            else:
                return False

        return True

    def completed_simualtion(self):
        count_completed = 0
        if self.target_simulation:
            for _ in self.swarm:
                if _.reached_goal(self.target_simulation):
                    count_completed = count_completed + 1 
        return count_completed/self.rate.in_num_swarm[self.rate.current_repetition]

    def rest_simulation(self):
        # new obstacles
        self.generate_obstacles()

        time = self.stop_watch - self.start_watch
        if self.time_executing > TIME_MAX_SIMULATION:
            time = "Goal not reached"
        self.rate.set_out(time)
            
        for _ in self.swarm:
            _.set_target(None)
            del _

        self.swarm = []
        self.start_watch = 0
        self.stop_watch = 0
        self.target_simulation = None
        self.create_swarm_uav(self.rate.in_num_swarm[self.rate.current_repetition])
        self.time_executing = 0 # Reset timer
        # set new random target for iteration
        target = vec2(uniform(100,SCREEN_WIDTH), uniform(100,SCREEN_HEIGHT))
        self.set_target(target)
