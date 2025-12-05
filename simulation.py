import time
import pygame
import math
import numpy as np
import matplotlib.pyplot as plt

from constants import *
from vehicle import Vehicle
from scan import ScanInterface
from state_machine import FiniteStateMachine, SeekState, SearchTargetState
from random import uniform
from obstacle import Obstacles
from utils import Npc_target
from grid import GridField

# New Managers
from experiment_manager import ExperimentManager
from display_manager import DisplayManager
from swarm_manager import SwarmManager

vec2 = pygame.math.Vector2

class Simulation(object):
    
    def __init__(self, display_manager: DisplayManager, experiment_manager: ExperimentManager):
        self.target_simulation = None
        self.display_manager = display_manager
        self.experiment_manager = experiment_manager
        
        self.start_watch = 0
        self.stop_watch = 0
        self.time_executing = 0 
        self.found = False
        
        # variables for obstacles
        self.obstacles = Obstacles(experiment_manager.in_num_obstacles[0], (SCREEN_WIDTH,SCREEN_HEIGHT))
        self.list_obst = []
        self.generate_obstacles()

        # Grid
        self.grid_field = GridField(RESOLUTION)

        # Swarm Manager
        self.swarm_manager = SwarmManager(display_manager)
        
        # Current simulations 
        self.targets_search = [] # memory of targets used in simulations

        # npc target 
        self.npc = Npc_target()
        self.all_sprites = pygame.sprite.Group()
        self.all_sprites.add(self.npc)

        # Create initial swarm
        self.swarm_manager.create_swarm(experiment_manager.in_num_swarm[0])

        # target 
        self.target_simulation = self.generate_new_random_target()
        self.targets_search.append(self.target_simulation)

    @property
    def swarm(self):
        """Proxy property for backward compatibility and ease of access."""
        return self.swarm_manager.swarm

    @property
    def rate(self):
        """Proxy property for backward compatibility."""
        return self.experiment_manager

    @property
    def screenSimulation(self):
        """Proxy property for backward compatibility."""
        return self.display_manager

    def generate_obstacles(self):
        # Generates obstacles
        self.obstacles.generate_obstacles()
        self.list_obst = self.obstacles.get_coordenates()

    def add_new_uav(self):
        self.swarm_manager.add_new_uav()
    
    def set_target(self, target, found=False):
        self.target_simulation = target
        self.swarm_manager.set_target(target, found)

    def set_time_target(self):
        self.experiment_manager.set_time_target(time.time() - self.start_watch)

    def set_target_using_search_pattern(self, target_simulation):
        '''
            IN TEST
            Set target area to be search  
        '''
        # saves global target
        self.target_simulation = target_simulation

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
            drone_target =  vec2(  cell_center[0], cell_center[1] )
            self.swarm[num_drones-1].set_target( drone_target ) 
            self.swarm[num_drones-1].mission_target = vec2(  drone_target )
            
            #print(f'drone: {num_drones} celula: {(r,c,step)} {vec2(  cell_center[0], cell_center[1] )}')

            table_search[r][c]  = num_drones
            num_drones -= 1
            
            # verifica se o passo não vai passar o limite de linhas da matriz
            if r < row - step:
                r += step
            else:
                r = 0 
                col_ = math.floor(col_/2)
                c+= col_
                
        #print(table_search)
        self.table_search = table_search
      
    def draw_obstacles(self):
        # draws the sprites of tree
        for _ in self.list_obst: 
            self.obstacles.all_sprites.draw(self.display_manager.world_surface)
            self.obstacles.all_sprites.update(_,0)
            pygame.draw.circle(self.display_manager.world_surface,(200, 200, 200), _, radius=RADIUS_OBSTACLES, width=1)
            pygame.draw.circle(self.display_manager.world_surface,(200, 200, 200), _, radius=RADIUS_OBSTACLES*1.6 + AVOID_DISTANCE, width=1)

    def draw_target(self):
        # draw target - npc
        if self.target_simulation: 
            self.all_sprites.draw(self.display_manager.world_surface)
            self.all_sprites.update(self.target_simulation,0)
            pygame.draw.circle(self.display_manager.world_surface, LIGHT_BLUE, self.target_simulation, RADIUS_TARGET, 2)

    def draw(self):
        #draw grid of visited celss
        self.grid_field.draw(self.display_manager.world_surface)
        # draw target - npc
        self.draw_target()
        # draw obstacles
        self.draw_obstacles()

    def run_simulation(self):
        # draw grid of visited cels, target and obstacles
        self.draw()

        # Target is Found: pass it to all drones
        if self.found:
            self.set_target(self.target_simulation, found = True)

        if self.start_watch == 0:
            self.start_watch = time.time()

        # for every drone, it will update the collision avoidace, aling the direction and draw current position in simuation
        # The scan method now delegates to swarm_manager.update()
        self.experiment_manager.in_algorithms[self.experiment_manager.current_repetition].scan(self, self.list_obst)
       
        self.time_executing += SAMPLE_TIME # count time of execution based on the sampling

        # check completition of simulation
        if self.completed_simulation() >= 0.8 and self.stop_watch == 0 or self.time_executing > TIME_MAX_SIMULATION:
            self.stop_watch = time.time()
            
            if self.experiment_manager and self.experiment_manager.next_simulation():
                self.rest_simulation()
            else:
                return False

        return True

    def completed_simulation(self):
        count_completed = 0

        if self.target_simulation:
            for _ in self.swarm:
                if _.reached_goal(self.target_simulation):
                    count_completed = count_completed + 1 
        return count_completed/self.experiment_manager.in_num_swarm[self.experiment_manager.current_repetition]

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
        self.obstacles.num_of_obstacles = self.experiment_manager.in_num_obstacles[self.experiment_manager.current_repetition]
        # Repeat scenario for new number of drones
        num_repet = self.experiment_manager.in_repetitions / len(self.experiment_manager.in_num_swarm)
        if self.experiment_manager.current_repetition > num_repet -1:
            self.obstacles.reset_seed()

        self.generate_obstacles()

        time = self.stop_watch - self.start_watch
        if self.time_executing > TIME_MAX_SIMULATION:
            time = "Goal not reached"
        self.experiment_manager.set_out(time, self.completed_simulation())
            
        # Clear swarm via manager
        self.swarm_manager.swarm = []
        
        self.start_watch = 0
        self.stop_watch = 0
        self.target_simulation = None
        serch_patter_for_iteration = self.experiment_manager.in_algorithms[self.experiment_manager.current_repetition].to_string()
        print(f'ITERATION USING: {serch_patter_for_iteration} ')
        
        # Recreate swarm
        self.swarm_manager.create_swarm(self.experiment_manager.in_num_swarm[self.experiment_manager.current_repetition], serch_patter_for_iteration)
        
        self.time_executing = 0 # Reset timer

        # set new random target for iteration
        target = self.generate_new_random_target()
        self.targets_search.append(target)
        self.set_target(target)

        # Prepare ALGORITHM TO SEARCH PATTERN
        self.experiment_manager.in_algorithms[self.experiment_manager.current_repetition].prepare_simulation(self, target)
        self.found = False