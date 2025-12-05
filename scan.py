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