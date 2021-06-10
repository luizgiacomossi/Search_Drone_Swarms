import time
import pygame
import math
from constants import *
from vehicle import Vehicle
from scan import ScanInterface
from state_machine import FiniteStateMachine, SeekState, SearchTargetState
from random import uniform
from obstacle import Obstacles
from utils import Npc_target
from grid import GridField
import numpy as np

vec2 = pygame.math.Vector2
##=========================
class RateSimulation(object):
    def __init__(self, in_repetitions, in_num_swarm, in_num_obstacles, in_algorithms):
        self.current_repetition = 0
        self.in_num_swarm = []
        self.in_num_obstacles = []
        self.in_algorithms = []
        
        # Inputs of Rate
        self.in_repetitions = in_repetitions * len(in_num_swarm) * len(in_num_obstacles) * len(in_algorithms)
        
        res = [[i, j, k] for i in enumerate(in_num_swarm) 
                         for j in enumerate(in_num_obstacles) 
                         for k in enumerate(in_algorithms) ]

        for r in enumerate(res):
            print(str(r))
            self.in_num_swarm.append(r[1][0][1])
            self.in_num_obstacles.append(r[1][1][1])
            self.in_algorithms.append(r[1][2][1])
        
        
        # Outputs of Rate
        self.out_time = []
        self.out_num_uav = []
        self.print_plan_rate()

    def set_out(self, out_time, out_num_uav):
        self.out_time.append(out_time)
        self.out_num_uav.append(out_num_uav)

    def next_simulation(self):
        if self.in_repetitions - 1 == self.current_repetition:
            return False
        else:
            self.current_repetition = self.current_repetition + 1
            self.print_simulation()
            return True

    def print_plan_rate(self):
        for idx in range(0, self.in_repetitions):
            print(f'{idx+1} - num_obstacles: {self.in_num_obstacles[idx]}, num_swarm : {self.in_num_swarm[idx]}, algorithm : {self.in_algorithms[idx].to_string()},')

    def print_simulation(self):
        return f'{self.current_repetition+1} - num_swarm: {self.in_num_swarm[self.current_repetition]}, num_obstacles: {self.in_num_obstacles[self.current_repetition]}, Algorithm: {self.in_algorithms[self.current_repetition].to_string()}'

    def print_simulation_idx(self, idx):
        return f'{idx+1} - Time: {time:.2f}, num_uav: {self.out_num_uav[idx]}, num_swarm: {self.in_num_swarm[idx]}, num_obstacles: {self.in_num_obstacles[idx]}'

    def print_rate(self):
        for idx, time in enumerate(self.out_time):
            print(f'{idx+1} - Time: {time}, num_uav: {self.out_num_uav[idx]}, num_swarm: {self.in_num_swarm[idx]}, num_obstacles: {self.in_num_obstacles[idx]}')

class ScreenSimulation(object):
    '''
        Class responsable to represent the screen variables
    ''' 
    def __init__(self):
        pygame.init()
        self.font16 = pygame.font.SysFont(None, 16)
        self.font20 = pygame.font.SysFont(None, 20)
        self.font24 = pygame.font.SysFont(None, 24)
        self.size = SCREEN_WIDTH, SCREEN_HEIGHT 
        self.clock = pygame.time.Clock()
        self.screen = pygame.display.set_mode(self.size)

class Simulation(object):
    
    def __init__(self, screenSimulation,rate:RateSimulation):
        self.target_simulation = None
        self.screenSimulation = screenSimulation
        self.start_watch = 0
        self.stop_watch = 0
        self.rate = rate
        self.time_executing = 0 
        self.found = False
        # variables for obstacles
        self.obstacles = Obstacles(rate.in_num_obstacles[0], (SCREEN_WIDTH,SCREEN_HEIGHT))
        self.list_obst = []
        self.generate_obstacles()

        # Grid
        self.grid_field = GridField(RESOLUTION)

        # state machines for each vehicle
        self.behaviors =[] 
        
        # Current simulations 
        self.swarm = []
        self.targets_search = []
        # npc target 
        self.npc = Npc_target()
        self.all_sprites = pygame.sprite.Group()
        self.all_sprites.add(self.npc)

        self.create_swarm_uav(rate.in_num_swarm[0])

        # target 
        self.target_simulation = self.generate_new_random_target()
        self.set_target_using_search_pattern(self.target_simulation)

    def generate_obstacles(self):
        # Generates obstacles
        self.obstacles.generate_obstacles()
        self.list_obst = self.obstacles.get_coordenates()

    def create_swarm_uav(self, num_swarm):
        # Create N simultaneous Drones
        for d in range(0, num_swarm):
            # voltar para seekstate para buscar com click do mouse e target conhecido
            self.behaviors.append( FiniteStateMachine( SearchTargetState() ) ) # Inicial state
            drone = Vehicle(uniform(0,100), uniform(0,100), self.behaviors[-1], self.screenSimulation.screen)
            self.swarm.append(drone)

    def add_new_uav(self):
        self.behaviors.append( FiniteStateMachine( SeekState() ) )
        drone = Vehicle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, self.behaviors[-1], self.screenSimulation.screen)

        drone.set_target(vec2(pygame.mouse.get_pos()[0],pygame.mouse.get_pos()[1]))
        self.append_uav(drone)
    
    def append_uav(self, drone):
        self.swarm.append(drone)

    def set_target(self, target, found=False):
        self.target_simulation = target
        for _ in self.swarm:
            _.set_target(target)
            if found == True:
                _.found = True

    def set_target_using_search_pattern(self, target):
        '''
            IN TEST
            Set target area to be search  
        '''
        # saves global target
        self.target_simulation = target

        # get #row and #col
        col = self.grid_field.cols
        row = self.grid_field.rows

        table_search = np.zeros((row,col))
        num_drones = len(self.swarm)

        step = math.ceil(row/num_drones)
        r=0
        c=0
        col_ = col 

        while num_drones > 0 :
            # self.swarm.set_target() - argumento é o target referente a celular
            # pegar a posicao do centro da celula
            cell_center = self.grid_field.cells[r][c].get_cell_center()
            self.swarm[num_drones-1].set_target( vec2(  cell_center[0], cell_center[1] )) 
            self.swarm[num_drones-1].mission_target = vec2(  cell_center[0], cell_center[1] )
            print(f'drone: {num_drones} celula: {(r,c,step)} {vec2(  cell_center[0], cell_center[1] )}')

            table_search[r][c]  = num_drones
            num_drones -= 1
            
            # verifica se o passo não vai passar o limite de linhas da matriz
            if r < row - step:
                r += step
                
            else:
                r = 0 
                col_ = math.floor(col_/2)
                c+= col_
                
        print(table_search)
        self.table_search = table_search
      
    def run_simulation(self):
        # draw grid of visited celss
        self.grid_field.draw(self.screenSimulation.screen)
        
        # Target is Found: pass it to all drones
        if self.found:
            self.set_target(self.target_simulation, found = True)

        # draw target - npc
        if self.target_simulation: 
            self.all_sprites.draw(self.screenSimulation.screen)
            self.all_sprites.update(self.target_simulation,0)
            pygame.draw.circle(self.screenSimulation.screen, LIGHT_BLUE, self.target_simulation, RADIUS_TARGET, 2)


        if self.start_watch == 0:
            self.start_watch = time.time()

        # for every drone, it will update the collision avoidace, aling the direction and draw current position in simuation
        self.rate.in_algorithms[self.rate.current_repetition].scan(self, self.list_obst)
       
        # draws the sprites of tree
        for _ in self.list_obst: 
            self.obstacles.all_sprites.draw(self.screenSimulation.screen)
            self.obstacles.all_sprites.update(_,0)
            pygame.draw.circle(self.screenSimulation.screen,(200, 200, 200), _, radius=RADIUS_OBSTACLES, width=1)
            pygame.draw.circle(self.screenSimulation.screen,(200, 200, 200), _, radius=RADIUS_OBSTACLES*1.6 + AVOID_DISTANCE, width=1)
            

        self.time_executing += SAMPLE_TIME # count time of execution based on the sampling
        #print(self.time_executing)

        # check completition of simulation
        if self.completed_simulation() >= 0.9 and self.stop_watch == 0 or self.time_executing > TIME_MAX_SIMULATION:
            self.stop_watch = time.time()
            
            if self.rate and self.rate.next_simulation():
                #pass
                self.rest_simulation()
            else:
                #pass
                return False

        return True

    def completed_simulation(self):
        count_completed = 0
        if self.target_simulation:
            for _ in self.swarm:
                if _.reached_goal(self.target_simulation):
                    count_completed = count_completed + 1 
        return count_completed/self.rate.in_num_swarm[self.rate.current_repetition]

    def generate_new_random_target(self):
        '''
            Generates valid random targets from a safe distance from obstacles
        '''
        found_valid_target= False 
        while not found_valid_target : 
            # generates new point
            target = vec2(uniform(SCREEN_WIDTH/3,SCREEN_WIDTH), uniform(100,SCREEN_HEIGHT))
            c=0
            #checks if it is valid
            for o in self.list_obst:
                # distance to obstacles
                d = target.distance_to(o)
                # check if ditance is not inside obstacle
                if d < RADIUS_OBSTACLES + RADIUS_TARGET:
                    c += 1
            # check counter
            if c == 0 :
                found_valid_target = True

        return target
        
    def rest_simulation(self):
        # reset grid 
        self.grid_field = GridField(RESOLUTION)

        # new obstacles
        self.obstacles.num_of_obstacles = self.rate.in_num_obstacles[self.rate.current_repetition]
        self.generate_obstacles()

        time = self.stop_watch - self.start_watch
        if self.time_executing > TIME_MAX_SIMULATION:
            time = "Goal not reached"
        self.rate.set_out(time, self.completed_simulation())
            
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
        target = self.generate_new_random_target()
        self.set_target(target)

        # TESTING NEW ALGORITHM TO SEARCH PATTERN
        self.set_target_using_search_pattern(target)
        self.found = False