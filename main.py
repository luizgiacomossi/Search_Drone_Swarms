import sys, pygame
from vehicle import Vehicle
from constants import *
import random 
import copy
from state_machine import FiniteStateMachine, SeekState, StayAtState, OvalState, Eight2State, ScanState
import threading # multithread

vec2 = pygame.math.Vector2
##=========================

pygame.init()
font20 = pygame.font.SysFont(None, 20)
font24 = pygame.font.SysFont(None, 24)
size = SCREEN_WIDTH, SCREEN_HEIGHT 
clock = pygame.time.Clock()
#bakcground
#bg = pygame.image.load("texture/Grass_01.png")
#bg = pygame.transform.scale(bg, (SCREEN_WIDTH, SCREEN_HEIGHT))
#bg.set_alpha(50)

screen = pygame.display.set_mode(size)
# defines initial target
target = vec2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2)

# state machines for each vehicle
behaviors =[] 
# Current simulations 
simulations = []

# Create N simultaneous Drones
for d in range(0, NUM_DRONES):
    behaviors.append( FiniteStateMachine( Eight2State() ) ) # Inicial state
    drone = Vehicle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, behaviors[-1], screen)
    drone.set_target(vec2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2))
    simulations.append(drone)

run = True
while run:
    # Draws at every dt
    clock.tick(FREQUENCY)

    # Pygame Events 
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()
        
        # Key 'd' pressed
        if event.type == pygame.KEYDOWN and event.key == pygame.K_d:
            print('apertei d')
            for _ in simulations:
                _.set_debug()

        # Mouse Clicked -> new taget or new Drone 
        if event.type == pygame.MOUSEBUTTONDOWN:
            # left button - New Target
            if pygame.mouse.get_pressed()[0] == True:
                target = vec2(pygame.mouse.get_pos()[0],pygame.mouse.get_pos()[1])
                # updates target of all simulations 
                for _ in simulations:
                    _.set_target(target)

            # right button - New Drone
            if pygame.mouse.get_pressed()[2] == True:              
                behaviors.append( FiniteStateMachine( SeekState() ) )
                drone = Vehicle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, behaviors[-1], screen)
                drone.set_target(vec2(pygame.mouse.get_pos()[0],pygame.mouse.get_pos()[1]))
                simulations.append(drone)

    # Background
    screen.fill(LIGHT_BLUE)
    #screen.blit(bg, (0,0))

    # draws target as a circle on screen
    pygame.draw.circle(screen, (100, 100, 100), target, RADIUS_TARGET, 2)

    # updates and draws all simulations  
    index = 0 # index is used to track current drone in the simulation list
    for _ in simulations:
        # checks if drones colided with eachother
        _.check_collision(simulations,index) 
        ## collision avoindance is not implemented yet
        #_.collision_avoidance(simulations,index)
        _.draw(screen) 
        _.update()
        index += 1
        # writes drone id
        img = font20.render(f'Drone {index}', True, BLUE)
        screen.blit(img, _.get_position()+(0,20))
        # writes drone actual behavior
        img = font20.render(_.behavior.get_current_state(), True, BLUE)
        screen.blit(img, _.get_position()+(0,30))

    # Writes the App name in screen
    img = font24.render('Paparazzi Mobility Model', True, BLUE)
    screen.blit(img, (20, 20))

    # Debug lines - only to assist the developer
    img = font24.render('Debug lines: '+ drone.get_debug(), True, BLUE)
    screen.blit(img, (20, 40))

    pygame.display.flip() 