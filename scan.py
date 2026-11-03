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
    """Row-based scanning strategy (parallel lawnmower/boustrophedon pattern)."""
    
    def to_string(self) -> str:
        return 'RowScan'
    
    def prepare_simulation(self, simulation, target) -> None:
        """Set up the simulation for row scanning.
        
        Args:
            simulation: The current simulation
            target: The target position
        """
        simulation.target_simulation = target
        num_drones = len(simulation.swarm)
        if num_drones == 0:
            return
            
        rows = simulation.grid_field.rows
        cols = simulation.grid_field.cols
        
        # Divide rows evenly across all drones in the swarm
        step = (rows + 1) / num_drones
        for i, drone in enumerate(simulation.swarm):
            r_start = int(round(i * step))
            r_end = int(round((i + 1) * step)) - 1
            r_end = max(r_start, min(r_end, rows))
            
            drone.row_start = r_start
            drone.row_end = r_end
            drone.current_row = r_start
            drone.direction = 1  # 1 = left-to-right, -1 = right-to-left
            drone.current_col = 0
            
            # Set initial target at start of drone's row
            cell_center = simulation.grid_field.cells[r_start][0].get_cell_center()
            target_pos = pygame.math.Vector2(cell_center[0], cell_center[1])
            drone.set_target(target_pos)
            drone.mission_target = pygame.math.Vector2(target_pos)
            drone.set_position_in_grid(0, r_start)

    def scan(self, simulation, list_obst) -> None:
        """Execute the row scanning algorithm.
        
        Args:
            simulation: The current simulation
            list_obst: List of obstacles
        """
        simulation.swarm_manager.update(simulation, list_obst)
        
        cols = simulation.grid_field.cols
        rows = simulation.grid_field.rows
        
        for drone in simulation.swarm:
            # Check if drone reached current target waypoint or needs a target
            target = getattr(drone, 'target', None)
            reached = False
            if target is None:
                reached = True
            elif hasattr(drone, 'reached_goal') and drone.reached_goal(target):
                reached = True
            elif (drone.location - target).length() < 35:
                reached = True
                
            if reached:
                # Ensure row scanning state attributes exist on drone
                if not hasattr(drone, 'row_start'):
                    drone.row_start = 0
                    drone.row_end = rows
                    drone.current_row = 0
                    drone.direction = 1
                    drone.current_col = 0

                # Advance column in current direction
                next_col = drone.current_col + drone.direction
                
                # Check if end of row is reached
                if next_col > cols or next_col < 0:
                    # Move to next row within this drone's assigned corridor
                    drone.current_row += 1
                    if drone.current_row > drone.row_end:
                        # Completed all assigned rows; loop back to start of corridor
                        drone.current_row = drone.row_start
                    
                    # Reverse horizontal direction
                    drone.direction *= -1
                    next_col = 0 if drone.direction == 1 else cols
                
                r = max(0, min(drone.current_row, rows))
                next_col = max(0, min(next_col, cols))
                
                # If obstacle ahead in this cell, skip along row to find next clear cell
                while 0 <= next_col <= cols and simulation.grid_field.cells[r][next_col].state == 2:  # 2 is OBSTACLE
                    next_col += drone.direction
                
                # If skipping went out of bounds, wrap to next row
                if next_col > cols or next_col < 0:
                    drone.current_row += 1
                    if drone.current_row > drone.row_end:
                        drone.current_row = drone.row_start
                    drone.direction *= -1
                    next_col = 0 if drone.direction == 1 else cols
                    r = max(0, min(drone.current_row, rows))
                    next_col = max(0, min(next_col, cols))
                
                drone.current_col = next_col
                cell_center = simulation.grid_field.cells[r][drone.current_col].get_cell_center()
                next_target = pygame.math.Vector2(cell_center[0], cell_center[1])
                drone.set_target(next_target)
                drone.mission_target = pygame.math.Vector2(next_target)


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
                    random.uniform(0, WORLD_WIDTH),
                    random.uniform(0, WORLD_HEIGHT)
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
            if (pos.x < margin and vel.x < 0) or (pos.x > WORLD_WIDTH - margin and vel.x > 0):
                vel.x *= -1
                bounce = True
                
            # Check vertical walls
            if (pos.y < margin and vel.y < 0) or (pos.y > WORLD_HEIGHT - margin and vel.y > 0):
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
                        random.uniform(0, WORLD_WIDTH),
                        random.uniform(0, WORLD_HEIGHT)
                    )
                
                if next_target_pos:
                    drone.set_target(next_target_pos)