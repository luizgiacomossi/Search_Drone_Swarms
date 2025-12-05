import sys
import random
import pygame
import matplotlib.pyplot as plt
from pygame.math import Vector2
from constants import *
from scan import DefineTargetScan, RowScan, MeshScan, SnookerScan
from obstacle import Obstacles
from simulation import Simulation
from display_manager import DisplayManager
from experiment_manager import ExperimentManager
from grid import GridField
import csv
from datetime import datetime
import traceback


class DroneSimulation:
    """Main class to manage the drone swarm simulation application."""
    
    def __init__(self):
        # Initialize pygame and screen
        pygame.init()
        self.display_manager = DisplayManager()
        
        # Load and scale background once
        self.background_image = self._load_background()
        
        # Set up initial target position
        initial_target = Vector2(
            random.uniform(0, SCREEN_WIDTH/2), 
            random.uniform(0, SCREEN_HEIGHT/2)
        )
        
        # Setup simulation with scan algorithms
        self.experiment_manager = ExperimentManager(
            10, 
            [10], 
            [20], 
            [DefineTargetScan(), RowScan(), MeshScan(), SnookerScan()]
        )
        
        self.simulation = Simulation(
            self.display_manager, 
            self.experiment_manager
        )
        self.simulation.set_target(initial_target)
        
    def _load_background(self):
        """Load and prepare the background image."""
        try:
            bg_image = pygame.image.load("models/texture/camouflage.png").convert()
            return pygame.transform.scale(bg_image, (SCREEN_WIDTH, SCREEN_HEIGHT))
        except pygame.error as e:
            print(f"Error loading background: {e}")
            # Create a fallback background if image loading fails
            fallback = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
            fallback.fill((50, 50, 50))  # Dark gray background
            return fallback
            
    def handle_events(self):
        """Process pygame events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
                
            # Toggle debug mode with 'd' key
            if event.type == pygame.KEYDOWN and event.key == pygame.K_d:
                for drone in self.simulation.swarm:
                    drone.set_debug()
                    
            # Handle mouse events
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = Vector2(pygame.mouse.get_pos())
                
                # Left-click: set new target
                if pygame.mouse.get_pressed()[0]:
                    self.simulation.set_target(mouse_pos)
                    
                # Right-click: add new drone
                if pygame.mouse.get_pressed()[2]:
                    self.simulation.add_new_uav()
            
            # Handle zoom
            if event.type == pygame.MOUSEWHEEL:
                self.display_manager.handle_zoom(event)
                    
        return True
    
    def render_ui(self):
        """Render UI elements and text."""
        # App title
        title = self.display_manager.font24.render(
            'Swarm Search using Drones', True, LIGHT_BLUE
        )
        self.display_manager.screen.blit(title, (20, 20))
        
        # Safely get current algorithm
        try:
            current_rep = self.simulation.rate.current_repetition
            alg_count = len(self.simulation.rate.in_algorithms)
            
            if alg_count > 0 and current_rep < alg_count:
                search = self.simulation.rate.in_algorithms[current_rep].to_string()
            else:
                search = "Unknown"
                
            search_text = self.display_manager.font24.render(
                f'Search using: {search}', True, LIGHT_BLUE
            )
            self.display_manager.screen.blit(search_text, (800, 20))
        except Exception as e:
            print(f"Error rendering search algorithm: {e}")
            # Fallback text if there's an error
            fallback = self.display_manager.font24.render(
                'Search algorithm: N/A', True, LIGHT_BLUE
            )
            self.display_manager.screen.blit(fallback, (800, 20))
        
        # Render mission stats
        self._render_mission_stats()
    
    def _render_mission_stats(self):
        """Render mission statistics with robust error handling."""
        try:
            for idx, time in enumerate(self.simulation.rate.out_time_mission):
                try:
                    # Get algorithm safely with bounds checking
                    alg_idx = self.simulation.rate.current_repetition
                    alg_count = len(self.simulation.rate.in_algorithms)
                    if alg_count > 0 and alg_idx < alg_count:
                        search = self.simulation.rate.in_algorithms[alg_idx].to_string()
                    else:
                        search = "Unknown"
                    
                    # Try to get simulation index info safely
                    try:
                        sim_idx_info = self.simulation.rate.print_simulation_idx(idx)
                    except:
                        sim_idx_info = "N/A"
                    
                    text = (
                        f'{idx+1} - Search: {search} - '
                        f'Scan Time: {time:.2f}, {sim_idx_info}'
                    )
                    img = self.display_manager.font20.render(text, True, LIGHT_BLUE)
                except:
                    # Simple fallback text
                    text = f'{idx+1} - Scan Time: {time:.2f}'
                    img = self.display_manager.font16.render(text, True, LIGHT_BLUE)
                    
                self.display_manager.screen.blit(img, (20, 20*(idx+2)))
        except Exception as e:
            # If even the rendering of mission stats fails
            print(f"Error rendering mission stats: {e}")
            img = self.display_manager.font16.render("Error displaying mission stats", True, LIGHT_BLUE)
            self.display_manager.screen.blit(img, (20, 40))
    
    def run(self):
        """Main simulation loop with robust error handling."""
        running = True
        
        try:
            while running:
                # Handle frame timing
                self.display_manager.clock.tick(FREQUENCY)
                # Process events
                running = self.handle_events()
                
                # Draw background
                # Draw background to world surface
                self.display_manager.world_surface.blit(self.background_image, (0, 0))
                
                # Update simulation state
                sim_running = self.simulation.run_simulation()
                if not sim_running:
                    running = False
                
    
                # Scale and blit world surface to screen
                scaled_surface = pygame.transform.scale(
                    self.display_manager.world_surface, 
                    (int(SCREEN_WIDTH * self.display_manager.zoom_level), 
                     int(SCREEN_HEIGHT * self.display_manager.zoom_level))
                )
                self.display_manager.screen.fill((50, 50, 50)) # Clear screen
                self.display_manager.screen.blit(scaled_surface, self.display_manager.offset)

                # Render UI elements (on top of everything, not zoomed)
                self.render_ui()
                
                # Update display
                pygame.display.flip()
                
            # Wait briefly when simulation ends
            pygame.time.wait(1000)
            
            # Always use our custom save method instead of the one in RateSimulation
            self._guaranteed_save_results()
            
        except Exception as e:
            print(f"Critical error in simulation: {e}")
            traceback.print_exc()
            # Try to save whatever data we have
            self._emergency_save_results()
        finally:
            # Always ensure pygame quits properly
            pygame.quit()
    
    def _guaranteed_save_results(self):
        """Custom save method that doesn't rely on the original save_csv."""
        if not SAVE_RESULTS:
            print("CSV saving disabled in constants.py")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"simulation_results_{timestamp}.csv"
        
        try:
            with open(filename, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Run", "Swarm Size", "Obstacles", "Algorithm", "Target Time", "Mission Time"])
                
                # Get all data safely
                mission_times = self.simulation.rate.out_time_mission
                target_times = getattr(self.simulation.rate, 'out_time_target', [])
                swarm_sizes = getattr(self.simulation.rate, 'in_num_swarm', [])
                obstacle_counts = getattr(self.simulation.rate, 'in_num_obstacles', [])
                algorithms = getattr(self.simulation.rate, 'in_algorithms', [])
                
                # Determine how many rows we have data for
                num_runs = len(mission_times)
                
                for i in range(num_runs):
                    # Get values safely with defaults if not available
                    swarm_size = swarm_sizes[i] if i < len(swarm_sizes) else "N/A"
                    obstacles = obstacle_counts[i] if i < len(obstacle_counts) else "N/A"
                    
                    # Handle algorithm names safely
                    if i < len(algorithms):
                        try:
                            algorithm = algorithms[i].to_string()
                        except:
                            algorithm = f"Algorithm_{i}"
                    else:
                        algorithm = "N/A"
                    
                    target_time = target_times[i] if i < len(target_times) else "N/A"
                    mission_time = mission_times[i]
                    
                    writer.writerow([i + 1, swarm_size, obstacles, algorithm, target_time, mission_time])
                
                print(f"Simulation results saved to {filename}")
                
                # Try to also run the original print_rate method if it exists
                try:
                    self.simulation.rate.print_rate()
                except:
                    pass
                    
        except Exception as e:
            print(f"Error in guaranteed save: {e}")
            traceback.print_exc()
            # Last resort emergency save
            self._emergency_save_results()
    
    def _emergency_save_results(self):
        """Absolute minimum save functionality as last resort."""
        if not SAVE_RESULTS:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"emergency_backup_{timestamp}.csv"
        
        try:
            with open(filename, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Run", "Mission Time"])
                
                # Try to get just the mission times
                try:
                    mission_times = self.simulation.rate.out_time_mission
                    for i, time in enumerate(mission_times):
                        writer.writerow([i + 1, time])
                except:
                    # Even if that fails, write a single entry noting the failure
                    writer.writerow([1, "Data retrieval failed"])
                
            print(f"Emergency backup saved to {filename}")
        except:
            print("All save attempts failed. No data could be saved.")


if __name__ == "__main__":
    try:
        sim = DroneSimulation()
        sim.run()
    except Exception as e:
        print(f"Fatal application error: {e}")
        traceback.print_exc()
        # Final attempt to ensure pygame quits
        try:
            pygame.quit()
        except:
            pass