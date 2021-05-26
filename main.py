import sys, pygame
from constants import *
import random 
import copy
from utils import FlowField
from scan import DefineTargetScan
from obstacle import Obstacles
from simulation import Simulation, ScreenSimulation, RateSimulation

vec2 = pygame.math.Vector2
##=========================
screenSimulation = ScreenSimulation()

# defines initial target
target = vec2(random.uniform(0,SCREEN_WIDTH/2), random.uniform(0,SCREEN_HEIGHT/2))

simulation = Simulation(screenSimulation, RateSimulation(5, [3,5], [DefineTargetScan()]))

run = True
while run:
    # Draws at every dt
    screenSimulation.clock.tick(FREQUENCY)

    # Pygame Events 
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()
        
        # Key 'd' pressed
        if event.type == pygame.KEYDOWN and event.key == pygame.K_d:
            for _ in simulation.swarm:
                _.set_debug()

        # Mouse Clicked -> new taget or new Drone 
        if event.type == pygame.MOUSEBUTTONDOWN:
            # left button - New Target
            if pygame.mouse.get_pressed()[0] == True:
                target = vec2(pygame.mouse.get_pos()[0],pygame.mouse.get_pos()[1])
                simulation.set_target(target)

            # right button - New Drone
            if pygame.mouse.get_pressed()[2] == True:
                simulation.add_new_uav()              
                
    # Background
    screenSimulation.screen.fill(LIGHT_BLUE)

    # draw grid
    #flow_field.draw(screen)

    # updates and draws all simulations  
    run = simulation.run_simulation()

    for idx, time in enumerate(simulation.rate.out_time):
        try:
            img = screenSimulation.font20.render(f'{idx+1} - Scan Time: {time:.2f}', True, BLUE)
        except:
            img = screenSimulation.font20.render(f'{idx+1} - Scan Time: {time}', True, BLUE)
        screenSimulation.screen.blit(img, (20, 20*(idx+2)))
        
    # Writes the App name in screen
    img = screenSimulation.font24.render('Swarm Search using Drones', True, BLUE)
    screenSimulation.screen.blit(img, (20, 20))

    # Debug lines - only to assist the developer
    #img = screenSimulation.font24.render('Debug lines: '+ drone.get_debug(), True, BLUE)
    #screenSimulation.screen.blit(img, (20, 40))

    pygame.display.flip()
    
    if not run:
        pygame.time.wait(5000) 