from constants import *
from typing import List, Optional
import pygame
import math
import random


class ScanInterface:
    """Base interface for scan algorithms in drone simulations."""
    
    def to_string(self) -> str:
        """Returns the name of the algorithm.
        
        Returns:
            str: The algorithm name
        """
        raise NotImplementedError("Subclasses must implement to_string()")
    
    def prepare_simulation(self, simulation, target) -> None:
        """Prepares the simulation with algorithm-specific settings.
        
        Args:
            simulation: The simulation object
            target: The target position
        """
        pass  # Default implementation does nothing

    def scan(self, simulation, list_obst) -> None:
        """Executes the scan algorithm.
        
        Args:
            simulation: The current simulation
            list_obst: List of obstacles
        """
        raise NotImplementedError("Subclasses must implement scan()")


class DefineTargetScan(ScanInterface):
    """Strategy when swarm knows the target location.
    Not for search, used for reference!
    """
    
    def to_string(self) -> str:
        return 'DefineTargetScan'

    def scan(self, simulation, list_obst) -> None:
        """Execute the standard target scan.
        
        Args:
            simulation: The current simulation
            list_obst: List of obstacles
        """
        # Delegate the update loop to the swarm manager
        simulation.swarm_manager.update(simulation, list_obst)


class RowScan(ScanInterface):
    """Row-based scanning strategy."""
    
    def to_string(self) -> str:
        return 'RowScan'
    
    def prepare_simulation(self, simulation, target) -> None:
        """Set up the simulation for row scanning.
        
        Args:
            simulation: The current simulation
            target: The target position
        """
        simulation.set_target_using_search_pattern(target)

    def scan(self, simulation, list_obst) -> None:
        """Execute the row scanning algorithm.
        
        Args:
            simulation: The current simulation
            list_obst: List of obstacles
        """
        # Delegate the update loop to the swarm manager
        simulation.swarm_manager.update(simulation, list_obst)

    def define_search_area(self) -> None:
        """Define the area to be searched."""
        pass


class RandoWalkScan(ScanInterface):
    """Random walk scanning strategy."""
    
    def to_string(self) -> str:
        return 'RandoWalkScan'

    def scan(self, simulation, list_obst) -> None:
        """Execute the random walk scanning algorithm.
        
        Args:
            simulation: The current simulation
            list_obst: List of obstacles
        """
        simulation.swarm_manager.update(simulation, list_obst)

        for drone in simulation.swarm:
            if drone.reached_goal(drone.target) or drone.target is None:
                # Generate new random target
                new_target = pygame.math.Vector2(
                    random.uniform(0, SCREEN_WIDTH),
                    random.uniform(0, SCREEN_HEIGHT)
                )
                drone.set_target(new_target)


class SnookerScan(ScanInterface):
    """Snooker-based scanning strategy."""
    
    def to_string(self) -> str:
        return 'SnookerScan'

    def prepare_simulation(self, simulation, target) -> None:
        """Set up the simulation for snooker scanning."""
        for drone in simulation.swarm:
            # Initialize with a random direction and a far target
            angle = random.uniform(0, 2 * math.pi)
            direction = pygame.math.Vector2(math.cos(angle), math.sin(angle))
            drone.set_target(drone.location + direction * 1000)

    def scan(self, simulation, list_obst) -> None:
        """Execute the snooker scanning algorithm.
        
        Args:
            simulation: The current simulation
            list_obst: List of obstacles
        """
        simulation.swarm_manager.update(simulation, list_obst)

        margin = SIZE_DRONE * 2
        
        for drone in simulation.swarm:
            pos = drone.location
            vel = drone.velocity
            bounce = False
            
            # Check horizontal walls
            if (pos.x < margin and vel.x < 0) or (pos.x > SCREEN_WIDTH - margin and vel.x > 0):
                vel.x *= -1
                bounce = True
                
            # Check vertical walls
            if (pos.y < margin and vel.y < 0) or (pos.y > SCREEN_HEIGHT - margin and vel.y > 0):
                vel.y *= -1
                bounce = True
            
            if bounce:
                # Update velocity and set new target in the reflected direction
                drone.velocity = vel
                if vel.length() > 0:
                    direction = vel.normalize()
                    drone.set_target(pos + direction * 1000)
            
            # If drone reached the "far" target without bouncing (e.g. large arena), keep going
            if drone.reached_goal(drone.target):
                if drone.velocity.length() > 0.1:
                    direction = drone.velocity.normalize()
                    drone.set_target(pos + direction * 1000)
                else:
                    # If stopped, pick random direction
                    angle = random.uniform(0, 2 * math.pi)
                    direction = pygame.math.Vector2(math.cos(angle), math.sin(angle))
                    drone.set_target(pos + direction * 1000)


class MeshScan(ScanInterface):
    """Mesh-based scanning strategy."""
    
    def to_string(self) -> str:
        return 'MeshScan'

    def prepare_simulation(self, simulation, target) -> None:
        """Set up the simulation for mesh scanning."""
        # Divide the grid among drones
        num_drones = len(simulation.swarm)
        rows = simulation.grid_field.rows
        cols = simulation.grid_field.cols
        
        # Calculate grid dimensions for drone assignment (e.g., 2x2 for 4 drones)
        grid_dim = int(math.ceil(math.sqrt(num_drones)))
        step_x = cols // grid_dim
        step_y = rows // grid_dim
        
        for i, drone in enumerate(simulation.swarm):
            # Determine starting cell for this drone
            r = (i // grid_dim) * step_y + step_y // 2
            c = (i % grid_dim) * step_x + step_x // 2
            
            # Ensure within bounds
            r = min(r, rows - 1)
            c = min(c, cols - 1)
            
            # Set initial target
            cell_center = simulation.grid_field.cells[r][c].get_cell_center()
            target_pos = pygame.math.Vector2(cell_center[0], cell_center[1])
            drone.set_target(target_pos)
            drone.set_position_in_grid(c, r)

    def scan(self, simulation, list_obst) -> None:
        """Execute the mesh scanning algorithm.
        
        Args:
            simulation: The current simulation
            list_obst: List of obstacles
        """
        simulation.swarm_manager.update(simulation, list_obst)
        
        for drone in simulation.swarm:
            if drone.reached_goal(drone.target):
                # Get current grid position
                col, row = drone.get_position_in_grid()
                
                # Find neighbors (indices)
                neighbors = []
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = col + dx, row + dy
                        if 0 <= nx < simulation.grid_field.cols and 0 <= ny < simulation.grid_field.rows:
                            neighbors.append((nx, ny))
                
                # Filter for unvisited neighbors
                unvisited = [n for n in neighbors if simulation.grid_field.cells[n[1]][n[0]].state == 0] # 0 is NOT_VISITED
                
                next_target_pos = None
                
                if unvisited:
                    # Pick a random unvisited neighbor
                    next_cell = random.choice(unvisited)
                    cell_center = simulation.grid_field.cells[next_cell[1]][next_cell[0]].get_cell_center()
                    next_target_pos = pygame.math.Vector2(cell_center[0], cell_center[1])
                elif neighbors:
                    # If all visited, pick any random neighbor to keep moving
                    next_cell = random.choice(neighbors)
                    cell_center = simulation.grid_field.cells[next_cell[1]][next_cell[0]].get_cell_center()
                    next_target_pos = pygame.math.Vector2(cell_center[0], cell_center[1])
                else:
                    # Fallback to random position if stuck
                    next_target_pos = pygame.math.Vector2(
                        random.uniform(0, SCREEN_WIDTH),
                        random.uniform(0, SCREEN_HEIGHT)
                    )
                
                if next_target_pos:
                    drone.set_target(next_target_pos)