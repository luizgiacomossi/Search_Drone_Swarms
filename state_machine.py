import random
from random import choices
from math import pi, atan2, inf
from constants import *
import numpy as np
import pygame 
import time
import copy 

NOT_VISITED = 0      
VISITED = 1
OBSTACLE = 2
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
        self.sampling_time = 3
        self.time_blocked = 0

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

     # New target from mouse click
        if agent.get_target():
            self.target = agent.get_target()
            agent.mission_target = agent.get_target()
            agent.set_target(None)


        dist = self.memory_last_position.distance_to(agent.get_position())
        
        # verifica se chegou
        d = self.target.distance_to(agent.get_position())

        if d <= RADIUS_TARGET and d > 3 :
            self.finished = True
            self.state_name = 'Done'

     # Verifica se terminou a execucao
        if self.finished == True:
            pass

        if dist < 30 and self.finished == False  :
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 2:
                pos_in_grid = agent.position_in_grid
                agent.grid_map.change_state_cell(pos_in_grid, OBSTACLE )

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
            state_machine.change_state(SeekState())  

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
        print('SearchTargetState')
        self.finished = False

        # Map resolution
        self.cols =int(SCREEN_WIDTH/RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT/RESOLUTION)  # Rows of the grid
        
        # waypoints to be followed
        self.waypoints = []
        self.next_waypoint = 0 

        # for checking if it is blocked
        self.memory_last_position = vec2(inf,inf)
        self.sampling_time = 2
        self.time_blocked = 0
        self.blocked = False

    def generate_waypoints(self):
        waypoints = [ vec2(0,0) ] # initial position
        #size of the grid
        cols = self.grid_map.get_size()

        global_coord = [ vec2(self.target[0],vec2(self.target[0])) ]
        # em coordenadas locais do grid
        # vai até o final
        waypoints.append( ( cols, 0 ) )
        # desce
        waypoints.append( ( 0, 1 ) )
        # volta
        waypoints.append( ( - cols , 0 ) )

        # converter para coordenadas globais
        for w in waypoints:
            global_coord.append( vec2(self.grid_map.get_cell_center(w)[0],self.grid_map.get_cell_center(w)[1] ) )
    
        return global_coord

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(SeekState())  
        
        try:
            #self.state_name = f'TARGET: {self.target}'
            pass
        except:
            pass

     # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 30 and self.finished == False :
            self.time_blocked += SAMPLE_TIME
            #self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 2:
                self.time_blocked = 0
                #state_machine.change_state(SearchTargetState())  
                self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
                

    def execute(self, agent):
        # logic to move drone to target
        try: # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target

        except: # nao tem, logo:
            self.target = agent.mission_target
            self.waypoints = agent.grid_map.get_sucessors( agent.position_in_grid )
            #print(self.waypoints)

        agent.arrive(self.target)

        self.time_executing +=SAMPLE_TIME
            
        if (self.target - agent.location).length() < 30 :
            #self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
            self.waypoints = agent.grid_map.get_sucessors( agent.position_in_grid )
            self.state_name = f'{self.waypoints}'
            if len(self.waypoints) > 0: # enquanto existem celulas nao visitadas na regiao
                targ = random.choice(self.waypoints)
                self.target = targ
            else: # random na tela para buscar ja que todas as celulas foram visitadas
                self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))

            #rint(f'EU IRIA PARA A CELULA : {agent.grid_map.get_cell_not_visited()}')

        # target is found by a drone in the swarm
        if agent.found:
            self.finished = True
            
        # Sampling location every T seconds
        if self.time_executing >=  self.sampling_time:
            self.time_executing = 0 
            self.memory_last_position = copy.deepcopy(agent.get_position())

 
class RandomSearchState(State):
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'SearchTargetState'
        self.time_executing = 0 #Variavel para contagem do tempo de execução 
        print('SearchTargetState')
        self.finished = False

        # Map resolution
        self.cols =int(SCREEN_WIDTH/RESOLUTION)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT/RESOLUTION)  # Rows of the grid
        
        # waypoints to be followed
        self.waypoints = []
        self.next_waypoint = 0 

        # for checking if it is blocked
        self.memory_last_position = vec2(inf,inf)
        self.sampling_time = 2
        self.time_blocked = 0
        self.blocked = False

    def generate_waypoints(self):
        waypoints = [ vec2(0,0) ] # initial position
        #size of the grid
        cols = self.grid_map.get_size()

        global_coord = [ vec2(self.target[0],vec2(self.target[0])) ]
        # em coordenadas locais do grid
        # vai até o final
        waypoints.append( ( cols, 0 ) )
        # desce
        waypoints.append( ( 0, 1 ) )
        # volta
        waypoints.append( ( - cols , 0 ) )

        # converter para coordenadas globais
        for w in waypoints:
            global_coord.append( vec2(self.grid_map.get_cell_center(w)[0],self.grid_map.get_cell_center(w)[1] ) )
    
        return global_coord

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # chegou ao waypoint
        if self.finished == True:
            state_machine.change_state(SeekState())  
        
        try:
            self.state_name = f'TARGET: {self.target}'
        except:
            pass

     # distancia percorrida desde a amostragem
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 70 and self.finished == False :
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 1:
                #state_machine.change_state(SearchTargetState())  
                #self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
                pass

    def execute(self, agent):
        # logic to move drone to target
        try: # verifica se o drone já te um target ou seja, uma coluna a cobrir
            self.target
        except: # nao tem, logo:
            self.target = agent.mission_target
            agent.grid_map.get_sucessors( agent.position_in_grid )

        agent.arrive(self.target)

        self.time_executing +=SAMPLE_TIME
            
        if (self.target - agent.location).length() < RADIUS_OBSTACLES*2 :
            self.target = vec2(random.uniform(0,SCREEN_WIDTH),random.uniform(0,SCREEN_HEIGHT))
            #rint(f'EU IRIA PARA A CELULA : {agent.grid_map.get_cell_not_visited()}')

        # target is found by a drone in the swarm
        if agent.found:
            self.finished = True
            
        # Sampling location every T seconds
        if self.time_executing >=  self.sampling_time:
            self.time_executing = 0 
            self.memory_last_position = copy.deepcopy(agent.get_position())

 

