import sys
import random
import pygame
import matplotlib.pyplot as plt
from pygame.math import Vector2
from constants import SCREEN_WIDTH, SCREEN_HEIGHT, WORLD_WIDTH, WORLD_HEIGHT, LIGHT_BLUE, FREQUENCY, SAVE_RESULTS
from scan import DefineTargetScan, RowScan, MeshScan, SnookerScan, RandoWalkScan
from obstacle import Obstacles
from simulation import Simulation
from display_manager import DisplayManager
from experiment_manager import ExperimentManager
from grid import GridField
import csv
from datetime import datetime
import traceback
import multiprocessing
from dashboard import run_dashboard


class DroneSimulation:
    """Main class to manage the drone swarm simulation application."""
    
    def __init__(self):
        # Initialize pygame and screen
        pygame.init()
        self.display_manager = DisplayManager()
        
        # Load and scale background once
        self.background_image = self._load_background()
        
        # Setup simulation with scan algorithms
        self.experiment_manager = ExperimentManager(
            10, 
            [10], 
            [20], 
            [DefineTargetScan(), RowScan()]
        )
        
        self.simulation = Simulation(
            self.display_manager, 
            self.experiment_manager
        )
        
        # Dashboard Setup
        self.stats_queue = multiprocessing.Queue()
        self.dashboard_process = multiprocessing.Process(
            target=run_dashboard, 
            args=(self.stats_queue,)
        )
        self.dashboard_process.daemon = True # Ensure it dies if main process dies forcefully
        self.dashboard_process.start()
        
        self.last_dashboard_update = 0
        self.last_run_index = 0
        
    def _load_background(self):
        """Load and prepare the background image."""
        try:
            bg_image = pygame.image.load("models/texture/camouflage.png").convert()
            return pygame.transform.scale(bg_image, (WORLD_WIDTH, WORLD_HEIGHT))
        except pygame.error as e:
            print(f"Error loading background: {e}")
            # Create a fallback background if image loading fails
            fallback = pygame.Surface((WORLD_WIDTH, WORLD_HEIGHT))
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
                world_pos = self.display_manager.screen_to_world(mouse_pos)
                
                # Left-click: set new target
                if pygame.mouse.get_pressed()[0]:
                    self.simulation.set_target(world_pos)
                    
                # Right-click: add new drone
                if pygame.mouse.get_pressed()[2]:
                    self.simulation.add_new_uav()
            
            # Handle zoom
            if event.type == pygame.MOUSEWHEEL:
                self.display_manager.handle_zoom(event)
                    
        return True
    
    def render_ui(self):
        """Render UI elements and text.
        
        NOTE: Visual legends and stats have been moved to the separate Dashboard window.
        This method is kept empty or minimal to keep the view clean.
        """
        pass
    
    def _render_mission_stats(self):
        """Deprecated: Stats are now in the dashboard."""
        pass


    def _update_dashboard(self):
        """Sends data to the dashboard process."""
        current_time = pygame.time.get_ticks()
        if current_time - self.last_dashboard_update < 100: # Update every 100ms
            return
            
        self.last_dashboard_update = current_time
        
        try:
            # Gather data
            idx = self.experiment_manager.current_repetition
            alg = "Unknown"
            if idx < len(self.experiment_manager.in_algorithms):
                alg = self.experiment_manager.in_algorithms[idx].to_string()
                
            swarm_size = self.experiment_manager.in_num_swarm[idx] if idx < len(self.experiment_manager.in_num_swarm) else 0
            obstacles = self.experiment_manager.in_num_obstacles[idx] if idx < len(self.experiment_manager.in_num_obstacles) else 0
            
            # Check for completed run to send history
            if idx > self.last_run_index:
                # The previous run (self.last_run_index) has finished
                try:
                    # Get results from the LAST run
                    # Note: We need to look back at the previous index (last_run_index) to get the correct inputs
                    prev_idx = self.last_run_index
                    
                    mission_time = self.experiment_manager.out_time_mission[-1]
                    
                    # Safely get inputs for that run
                    prev_alg = "Unknown"
                    if prev_idx < len(self.experiment_manager.in_algorithms):
                        prev_alg = self.experiment_manager.in_algorithms[prev_idx].to_string()
                        
                    prev_swarm = self.experiment_manager.in_num_swarm[prev_idx] if prev_idx < len(self.experiment_manager.in_num_swarm) else 0
                    prev_obst = self.experiment_manager.in_num_obstacles[prev_idx] if prev_idx < len(self.experiment_manager.in_num_obstacles) else 0
                    
                    # Get coverage if available
                    prev_coverage = 0.0
                    if hasattr(self.experiment_manager, 'out_coverage') and len(self.experiment_manager.out_coverage) > prev_idx:
                         # The list might store strings if "Goal not reached", but coverage should be float.
                         # Wait, set_out appends sequentially, so out_coverage[-1] should correspond to this run.
                         prev_coverage = self.experiment_manager.out_coverage[-1]
                    
                    entry = f"Run {prev_idx + 1}: {prev_alg}, {prev_swarm} UAVs, {prev_obst} Obs, Cov: {prev_coverage*100:.1f}%, Time: {mission_time:.2f}s"
                    
                    self.stats_queue.put({'history_entry': entry})
                except IndexError:
                    pass
                self.last_run_index = idx
            
            data = {
                'algorithm': alg,
                'swarm_size': swarm_size,
                'obstacles': obstacles,
                'run_index': idx + 1,
                'time': self.simulation.time_executing,
                'progress': self.simulation.completed_simulation(),
                'coverage': self.simulation.grid_field.get_coverage_percentage()
            }
            
            self.stats_queue.put(data)
            
        except Exception as e:
            print(f"Dashboard update error: {e}")

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
                
                # Update Dashboard
                self._update_dashboard()
                
                # Smart rendering to avoid scaling huge surfaces
                zoom = self.display_manager.zoom_level
                offset = self.display_manager.offset
                
                self.display_manager.screen.fill((50, 50, 50)) # Clear screen

                # If zoomed in enough that world covers screen (or mostly)
                # We crop the visible area and scale ONLY that.
                # This prevents creating massive surfaces (e.g. 20k x 10k pixels) when zooming in.
                if zoom > 1.0:
                    # Calculate visible world area
                    # world_x = (screen_x - offset_x) / zoom
                    wx = -offset.x / zoom
                    wy = -offset.y / zoom
                    ww = SCREEN_WIDTH / zoom
                    wh = SCREEN_HEIGHT / zoom
                    
                    # Clamp to world bounds to avoid errors
                    wx = max(0, wx)
                    wy = max(0, wy)
                    ww = min(ww, WORLD_WIDTH - wx)
                    wh = min(wh, WORLD_HEIGHT - wy)
                    
                    if ww > 0 and wh > 0:
                        try:
                            subsurf = self.display_manager.world_surface.subsurface(pygame.Rect(wx, wy, ww, wh))
                            scaled_surf = pygame.transform.scale(subsurf, (int(ww * zoom), int(wh * zoom)))
                            self.display_manager.screen.blit(scaled_surf, (0, 0))
                        except Exception as e:
                            # Fallback if subsurface fails (e.g. out of bounds due to float precision)
                            print(f"Render error: {e}")
                            pass
                else:
                    # Zoomed out or normal: scale the whole world (it's small enough)
                    scaled_surface = pygame.transform.scale(
                        self.display_manager.world_surface, 
                        (int(WORLD_WIDTH * zoom), 
                         int(WORLD_HEIGHT * zoom))
                    )
                    self.display_manager.screen.blit(scaled_surface, offset)

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
            # Kill Dashboard
            if self.dashboard_process.is_alive():
                self.dashboard_process.terminate()
                self.dashboard_process.join()
                
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
    # Required for multiprocessing on some platforms / freeze support
    multiprocessing.freeze_support()
    
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