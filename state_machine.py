import random
from random import choices
from math import pi, atan2, inf
from constants import *
import numpy as np
import pygame 
import time
import copy 

vec2 = pygame.math.Vector2

def get_random_state():
    """
        This method will sample random state based on the list of states and their probabilities
        
        :return: The name of the state
        :rtype: string
    """
    states = ['SeekState','StayAtState', 'Eight2State','OvalState', 'ScanState' ] 
    probabilities = [0.05, 0.3, 0.05, 0.3, 0.3]

    x = np.random.choice(states, p =probabilities)
    return x

class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)

    def get_current_state(self):
        return self.state.state_name

class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

class SeekState(State):
    """
        Drone will seek target  
    """
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'SeekState'
        self.time_executing = 0 #Variavel para contagem do tempo de execução 
        print('Seek')
        self.finished = False
        self.memory_last_position = vec2(inf,inf)
        self.sampling_time = 1
        self.time_blocked = 0

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

     # New target from mouse click
        if agent.get_target():
            self.target = agent.get_target()
            agent.mission_target = agent.get_target()
            agent.set_target(None)

     # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        # verifica se chegou
        if dist<= RADIUS_TARGET and dist > 3 :
            self.finished = True
        else: # nao chegou
            self.state_name = f'buscando target: {self.target}'

     # Verifica se terminou a execucao
        if self.finished == True:
            #self.state_name = 'Cheguei'
            state_machine.change_state(SeekState())  

        if dist < 60 and self.finished == False :
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'TRAVADO: {self.time_blocked:.2f}'
            if self.time_blocked > 2:
                state_machine.change_state(GoToClosestDroneState())  

    def execute(self, agent):
        # logic to move drone to target
        try:
            self.target
        except:
            self.target = agent.mission_target

        agent.arrive(self.target)
        self.time_executing += SAMPLE_TIME

        # Sampling location every T seconds
        if self.time_executing >=  self.sampling_time:
            self.time_executing = 0 
            self.memory_last_position = copy.deepcopy(agent.get_position())

class GoToClosestDroneState(State):
    """
        Drone will seek closest drone in swarm  
    """
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'GoToClosesDroneState'
        self.time_executing = 0 #Variavel para contagem do tempo de execução 
        print('GoToClosesDroneState')
        self.finished = False

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # New target from mouse click
        if agent.get_target():
            self.target = agent.get_target()
            agent.set_target(None)
            self.sequence = 0 # reinicia movimento

        if self.time_executing > 3:
            state_machine.change_state(RandomTargetState()) 

        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(SeekState())  
 
             
    def execute(self, agent):
        # logic to move drone to target

        self.target = agent.get_closest_drone()

        agent.arrive(self.target)
        self.time_executing +=SAMPLE_TIME
        
        if (self.target - agent.location).length() < SIZE_DRONE*2*1.4 :
            self.finished = True
    
class RandomTargetState(State):
    """
        Drone will seek a random target to unblock as last resort
    """
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'RandomTargetState'
        self.time_executing = 0 #Variavel para contagem do tempo de execução 
        print('RandomTargetState')
        self.finished = False

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # chegou ao waypoint
        if self.finished == True:
                state_machine.change_state(SeekState())  
 
             
    def execute(self, agent):
        # logic to move drone to target
        try:
            self.target
        except:
            random_position = vec2(random.uniform(-400,400),random.uniform(-400,400))
            self.target = agent.mission_target+ random_position

        agent.arrive(self.target)
        self.time_executing +=SAMPLE_TIME
        
        if (self.target - agent.location).length() < 10 or self.time_executing > 3:
            self.finished = True

class SearchTargetState(State):
    """
        Drone will seek a random target to unblock as last resort
    """
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'SearchTargetState'
        self.time_executing = 0 #Variavel para contagem do tempo de execução 
        #print('SearchTargetState')
        self.finished = False

        # Map resolution
        self.cols =int(SCREEN_WIDTH/RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT/RESOLUTION)  # Rows of the grid
        
        # waypoints to be followed
        self.waypoints = [ vec2(0,0) , vec2( SCREEN_WIDTH - 100, 0 ) , vec2( 0, RESOLUTION ) ]
        self.next_waypoint = 0 

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(SeekState())  
 
             
    def execute(self, agent):
        # logic to move drone to target
        try: # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target
        except: # nao tem, logo:
            self.target = agent.mission_target
            self.grid_map = agent.grid_map


        self.target = self.waypoints[self.next_waypoint] + agent.mission_target
        self.state_name = f'buscando target: {self.target}'

        agent.arrive(self.target)
        self.time_executing +=SAMPLE_TIME
        
        if (self.target - agent.location).length() < RADIUS_OBSTACLES :
            self.next_waypoint += 1
            if self.next_waypoint > len(self.waypoints) -1:
                self.next_waypoint = 0 

        if agent.found:
            self.finished = True
