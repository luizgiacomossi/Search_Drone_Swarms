import pygame
from random import uniform
from typing import List
from vehicle import Vehicle
from state_machine import FiniteStateMachine, SeekState, SearchTargetState
from constants import SCREEN_WIDTH, SCREEN_HEIGHT, LIGHT_BLUE, RESOLUTION
from quadtree import Quadtree, Rect

vec2 = pygame.math.Vector2

class SwarmManager:
    def __init__(self, display_manager):
        self.swarm: List[Vehicle] = []
        self.behaviors = []
        self.display_manager = display_manager

    def create_swarm(self, num_swarm, search_pattern='DefineTargetScan'):
        self.swarm = []
        self.behaviors = []
        for d in range(0, num_swarm):
            # Seek state: se tem o target inicialmente:
            if search_pattern == 'RowScan':
                self.behaviors.append( FiniteStateMachine( SearchTargetState() ) ) # Inicial state
            else:
                self.behaviors.append( FiniteStateMachine( SeekState() ) ) # Inicial state

            drone = Vehicle(uniform(0,100), uniform(0,100), self.behaviors[-1], self.display_manager.world_surface)
            self.swarm.append(drone)

    def add_new_uav(self):
        self.behaviors.append( FiniteStateMachine( SeekState() ) )
        drone = Vehicle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, self.behaviors[-1], self.display_manager.world_surface)

        # Convert mouse position to world coordinates for target
        mouse_pos = vec2(pygame.mouse.get_pos()[0], pygame.mouse.get_pos()[1])
        world_pos = self.display_manager.screen_to_world(mouse_pos)
        
        drone.set_target(world_pos)
        self.swarm.append(drone)

    def set_target(self, target, found=False):
        for drone in self.swarm:
            drone.set_target(target)
            if found:
                drone.found = True

    def update(self, simulation, list_obst):
        """
        Updates the entire swarm:
        1. Builds Quadtree
        2. Updates grid
        3. Updates physics/behavior (collision avoidance, etc.)
        4. Draws drones
        """
        # Build Quadtree
        boundary = Rect(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, SCREEN_WIDTH/2, SCREEN_HEIGHT/2)
        quadtree = Quadtree(boundary, 4) # Capacity 4
        
        for drone in self.swarm:
            quadtree.insert(drone.location, drone)

        target_found = False
        
        for index, drone in enumerate(self.swarm):
            # Update grid with drone's position
            self._update_grid(drone, simulation)
            
            # Update drone physics and behavior
            drone.align_direction_with_swarm(quadtree, index)
            drone.collision_avoidance(quadtree, list_obst, index) 
            drone.update()
            
            # Draw drone
            drone.draw(self.display_manager.world_surface)
            self._draw_legend(drone, index)
            
            # Check if drone reached the target
            if simulation.target_simulation and drone.reached_goal(simulation.target_simulation):
                simulation.found = True
                simulation.set_time_target()
                target_found = True
                
        return target_found

    def _update_grid(self, drone, simulation) -> None:
        """Updates the grid with the drone's current position."""
        # Calculate grid position
        col, row = self._get_grid_position(drone)

        # Update grid and drone state
        simulation.grid_field.change_state_cell((col, row))
        drone.set_position_in_grid(col, row)
        drone.save_grid(simulation.grid_field)

    def _get_grid_position(self, drone) -> tuple:
        """Calculates the grid position of a drone."""
        p = drone.get_position()
        col = int(p.x / RESOLUTION) 
        row = int(p.y / RESOLUTION)
        return col, row

    def _draw_legend(self, drone, index) -> None:
        """Draws information text beneath the drone."""
        position = drone.get_position()
        screen = self.display_manager.world_surface
        font20 = self.display_manager.font20
        font16 = self.display_manager.font16
        
        # Drone ID
        drone_id_text = font20.render(f'Drone {index+1}', True, LIGHT_BLUE)
        screen.blit(drone_id_text, position + (0, 20))
        
        # Current behavior
        behavior_text = font16.render(drone.behavior.get_current_state(), True, LIGHT_BLUE)
        screen.blit(behavior_text, position + (0, 30))
        
        # Current grid position
        col, row = self._get_grid_position(drone)
        pos_text = font16.render(f'Pos:{col},{row}', True, LIGHT_BLUE)
        screen.blit(pos_text, position + (0, 40))
