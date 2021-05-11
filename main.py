import sys, pygame
from constants import *
import random 
import copy
from utils import FlowField
from obstacle import Obstacles
from simulation import Simulation, ScreenSimulation
vec2 = pygame.math.Vector2
##=========================
screenSimulation = ScreenSimulation()

# defines initial target
target = vec2(random.uniform(0,SCREEN_WIDTH/2), random.uniform(0,SCREEN_HEIGHT/2))

# Generates obstacles
list_obst = []
obst = Obstacles(10, (SCREEN_WIDTH,SCREEN_HEIGHT))
obst.generate_obstacles()
# To generate obstacles, uncomment following command
#list_obst = obst.get_coordenates()

#creates flow field - not used neither fully implemented, flow field can be used as wind
#flow_field = FlowField(RESOLUTION)

simulation = Simulation(screenSimulation)
simulation.create_swarm_uav(NUM_DRONES)

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
    # Draws obstacles:
    for _ in list_obst:
        pygame.draw.circle(screenSimulation.screen,(100, 100, 100), _, radius=80)

    # draw grid
    #flow_field.draw(screen)

    # draws target as a circle on screen
    if target:
        pygame.draw.circle(screenSimulation.screen, (100, 100, 100), target, RADIUS_TARGET, 2)

    # updates and draws all simulations  
    simulation.run_simulation(list_obst)


    # Writes the App name in screen
    img = screenSimulation.font24.render('Swarm Search using Drones', True, BLUE)
    screenSimulation.screen.blit(img, (20, 20))

    # Debug lines - only to assist the developer
    #img = screenSimulation.font24.render('Debug lines: '+ drone.get_debug(), True, BLUE)
    #screenSimulation.screen.blit(img, (20, 40))

    pygame.display.flip() 