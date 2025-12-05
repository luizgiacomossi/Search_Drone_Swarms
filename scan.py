from constants import *
from typing import List, Optional


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

    def update_drone(self, drone, simulation, list_obst, index) -> None:
        """Updates a drone's behavior, position, and renders it.
        
        This method aligns the drone with the swarm, performs collision avoidance,
        updates velocity and position, and draws the drone on screen.
        
        Args:
            drone: The drone object to update
            simulation: The current simulation
            list_obst: List of obstacles
            index: Index of the drone in the swarm
        """
        drone.align_direction_with_swarm(simulation.swarm, index)
        drone.collision_avoidance(simulation.swarm, list_obst, index) 
        drone.update()
        drone.draw(simulation.screenSimulation.world_surface) 
    
    def draw_legend(self, drone, simulation, index) -> None:
        """Draws information text beneath the drone.
        
        Args:
            drone: The drone object
            simulation: The current simulation
            index: Index of the drone in the swarm
        """
        position = drone.get_position()
        screen = simulation.screenSimulation.world_surface
        font20 = simulation.screenSimulation.font20
        font16 = simulation.screenSimulation.font16
        
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

    def _get_grid_position(self, drone) -> tuple:
        """Calculates the grid position of a drone.
        
        Args:
            drone: The drone object
            
        Returns:
            tuple: (column, row) position in the grid
        """
        p = drone.get_position()
        col = int(p.x / RESOLUTION) 
        row = int(p.y / RESOLUTION)
        return col, row

    def update_grid(self, drone, simulation) -> None:
        """Updates the grid with the drone's current position.
        
        Args:
            drone: The drone object
            simulation: The current simulation
        """
        # Calculate grid position
        col, row = self._get_grid_position(drone)

        # Update grid and drone state
        simulation.grid_field.change_state_cell((col, row))
        drone.set_position_in_grid(col, row)
        drone.save_grid(simulation.grid_field)

    def scan(self, simulation, list_obst) -> None:
        """Executes the scan algorithm.
        
        Args:
            simulation: The current simulation
            list_obst: List of obstacles
        """
        raise NotImplementedError("Subclasses must implement scan()")
    
    def process_swarm(self, simulation, list_obst) -> bool:
        """Process the entire swarm with the scanning algorithm.
        
        This is a helper method that handles common operations for all drones.
        
        Args:
            simulation: The current simulation
            list_obst: List of obstacles
            
        Returns:
            bool: True if target was found, False otherwise
        """
        target_found = False
        
        for index, drone in enumerate(simulation.swarm):
            # Update grid with drone's position
            self.update_grid(drone, simulation)
            
            # Handle drone movement and collision avoidance
            self.update_drone(drone, simulation, list_obst, index)
            
            # Draw the drone's information
            self.draw_legend(drone, simulation, index)
            
            # Check if drone reached the target
            if drone.reached_goal(simulation.target_simulation):
                simulation.found = True
                simulation.set_time_target()
                target_found = True
                
        return target_found


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
        self.process_swarm(simulation, list_obst)


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
        self.process_swarm(simulation, list_obst)

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
        # Implementation to be added
        pass


class SnookerScan(ScanInterface):
    """Snooker-based scanning strategy."""
    
    def to_string(self) -> str:
        return 'SnookerScan'

    def scan(self, simulation, list_obst) -> None:
        """Execute the snooker scanning algorithm.
        
        Args:
            simulation: The current simulation
            list_obst: List of obstacles
        """
        # Implementation to be added
        pass


class MeshScan(ScanInterface):
    """Mesh-based scanning strategy."""
    
    def to_string(self) -> str:
        return 'MeshScan'

    def scan(self, simulation, list_obst) -> None:
        """Execute the mesh scanning algorithm.
        
        Args:
            simulation: The current simulation
            list_obst: List of obstacles
        """
        # Implementation to be added
        pass