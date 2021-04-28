import random
from random import choices
from math import pi, atan2
from constants import *
import numpy as np
import pygame 
import time

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

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # New target from mouse click
        if agent.get_target():
            self.target = agent.get_target()
            agent.set_target(None)
            self.sequence = 0 # reinicia movimento

        # chegou ao waypoint
        # if self.finished == True:
        #     next_state = get_random_state()
        #     if next_state == 'SeekState':
        #         state_machine.change_state(SeekState())  
        #     elif next_state == 'StayAtState':
        #         state_machine.change_state(StayAtState())  
        #     elif next_state == 'Eight2State':
        #         state_machine.change_state(Eight2State())  
        #     elif next_state == 'OvalState':
        #         state_machine.change_state(OvalState())  
        #     elif next_state == 'ScanState':
        #         state_machine.change_state(ScanState())  
             
    def execute(self, agent):
        # logic to move drone to target
        try:
            self.target
        except:
            self.target = agent.get_position()

        agent.seek(self.target)
        self.time_executing +=1
        if (self.target - agent.location).length() < 10 and self.time_executing > 300:
            self.finished = True

class StayAtState(State):
    """
        Drone will orbit a target
    """ 
    def __init__(self):
        self.state_name = 'StayAt'
        #  initialization code
        self.time_executing = 0 #Variavel para contagem do tempo de execução 
        print('StayAt')
        
    def check_transition(self, agent, state_machine):
        #  logic to check and execute state transition

        # New target from mouse click
        if agent.get_target():
            self.target = agent.get_target()
            agent.set_target(None)
            self.sequence = 0 # reinicia movimento

        # Arbitrary runtime transition-> next state is random
        if self.time_executing > 6000:
            next_state = get_random_state()
            if next_state == 'SeekState':
                state_machine.change_state(SeekState())  
            elif next_state == 'StayAtState':
                state_machine.change_state(StayAtState())  
            elif next_state == 'Eight2State':
                state_machine.change_state(Eight2State())  
            elif next_state == 'OvalState':
                state_machine.change_state(OvalState())  
            elif next_state == 'ScanState':
                state_machine.change_state(ScanState())   
        
    def execute(self, agent):
        # logic to move drone to target
        try:
            self.target
        except:
            self.target = vec2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2) 

        self.time_executing += 1
        # defines target 
        agent.seek_around(self.target)

class OvalState(State):
    """
         Drone will perform a Oval path
    """ 
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'Oval'
        self.theta = 0 #Variavel para contagem do angulo atual  
        print('OvalState')
        self.target = vec2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2)
        self.sequence = 0
        self.finished = False

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        #if self.time_executing > 300:
            #state_machine.change_state(SeekState())  
        # Transição Colidiu -> Mover para estado: Go Back
        if agent.get_target():
            self.target = agent.get_target()
            agent.set_target(None)
            self.sequence = 0 # reinicia movimento
        
        if self.finished == True:
            next_state = get_random_state()
            if next_state == 'SeekState':
                state_machine.change_state(SeekState())  
            elif next_state == 'StayAtState':
                state_machine.change_state(StayAtState())  
            elif next_state == 'Eight2State':
                state_machine.change_state(Eight2State())  
            elif next_state == 'OvalState':
                state_machine.change_state(OvalState())  
            elif next_state == 'ScanState':
                state_machine.change_state(ScanState())  

    def execute(self, agent):
        # logic to move drone to target
        pos = self.target - agent.get_position() 
        #print(f'distancia até target {pos.length()}')
        ang = atan2(pos.y,pos.x)
        e = FORWARD_SPEED / RADIUS_TARGET
        if pi/2 - e  < ang < pi/2 + e  and self.sequence == 0 :
            #time.sleep(1)
            self.sequence = 1
            self.target = agent.get_position() + vec2(300,0)
        # inicia girando até 90 graus

        if self.sequence == 0:
            agent.seek_around(self.target)

        # sai na tangente aos 90 graus
        if self.sequence == 1:
            agent.seek(self.target)
            if pos.length() < 5:
                self.target = agent.get_position() + vec2(0,RADIUS_TARGET)
                self.sequence = 2
                
        # rotaciona em novo taget        
        if self.sequence == 2:
            agent.seek_around(self.target)
                # se esta em -pi/2
            if  -pi/2 - e  < ang < -pi/2 + e :
                self.target = agent.get_position() + vec2(-300,0)
                self.sequence = 3

        # vai até ang -pi/2 do target
        if self.sequence == 3:
                agent.seek(self.target)
                # chegou ao ponto, recomeça
                if pos.length() < 5:
                    self.target = agent.get_position() + vec2(0, -RADIUS_TARGET)
                    self.sequence = 0
                    self.finished = True

class Eight2State(State):
    """
        Drone will perform a eight movement
    """ 
    def __init__(self):
        self.state_name = 'Eight2'
        # Todo: add initialization code
        self.theta = 0 #Variavel para contagem do angulo atual  
        print('EightState2')
        self.target = vec2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2)
        self.sequence = 0
        self.finished = 0 

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # Transição Colidiu -> Mover para estado: Go Back
        if agent.get_target():
            self.target = agent.get_target()
            agent.set_target(None)
            self.sequence = 0 # reinicia movimento

        # when executed 3 times -> new random state
        if self.finished == 3: 
            next_state = get_random_state()
            if next_state == 'SeekState':
                state_machine.change_state(SeekState())  
            elif next_state == 'StayAtState':
                state_machine.change_state(StayAtState())  
            elif next_state == 'Eight2State':
                state_machine.change_state(Eight2State())  
            elif next_state == 'OvalState':
                state_machine.change_state(OvalState())  
            elif next_state == 'ScanState':
                state_machine.change_state(ScanState())   

    def execute(self, agent):
        # logic to move drone to target
        pos = self.target - agent.get_position() 
        ang = atan2(pos.y,pos.x)
        ang_corte = pi/2
        # step error
        e = FORWARD_SPEED / RADIUS_TARGET
        if self.sequence == 0 :
            # inicia girando até 90 graus
            agent.seek_around(self.target)
            if ang_corte - e < ang < ang_corte + e:
                self.sequence = 1
                # way point 2
                self.target = agent.get_position() + vec2(2*RADIUS_TARGET,2*RADIUS_TARGET)
        
        # Primeira reta do X
        if self.sequence == 1:
            agent.seek(self.target)
            if pos.length() < 5:
                self.target = agent.get_position() + vec2(0,-RADIUS_TARGET)
                self.sequence = 2
                
        # faz meia volta no segundo target       
        if self.sequence == 2:
            agent.seek_around(self.target)
                # se esta em -pi/2
            if  ang_corte - e < ang < ang_corte + e:
                self.target = agent.get_position() + vec2(-2*RADIUS_TARGET,+2*RADIUS_TARGET)
                self.sequence = 3

        # vai até ang -pi/2 do target
        if self.sequence == 3:
                agent.seek(self.target)
                # chegou ao ponto, recomeça
                if pos.length() < 5:
                    self.target = agent.get_position() + vec2(0, -RADIUS_TARGET)
                    self.sequence = 0
                    self.finished +=1

class ScanState(State):
    """
        Drone will Scan a area 
    """ 
    def __init__(self):
        # Todo: add initialization code
        self.state_name = 'ScanState'
        self.theta = 0 #Variavel para contagem do angulo atual  
        print('ScanState')
        self.target = vec2(300, 60)
        self.sequence = 0
        self.finished = False
        self.radius = 50 
        d = 500 # distance for a straigth line
        # sequence of movements of behavior
        self.waypoints = [ vec2(0,0), #origin 
                           vec2(d, 0), 
                           vec2(0,self.radius),
                           vec2(-d,0), 
                           vec2(0,self.radius),
                           vec2(d, 0), 
                           vec2(0, self.radius),
                           vec2(-d, 0),
                           vec2(0, self.radius),
                           vec2(d,0)]

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        #if self.time_executing > 300:
            #state_machine.change_state(SeekState())  

        # Transição novo target do mouse
        if agent.get_target():
            self.target = agent.get_target()
            agent.set_target(None)
            self.sequence = 0 # reinicia movimento

        if self.finished == True:
            next_state = get_random_state()
            if next_state == 'SeekState':
                state_machine.change_state(SeekState())  
            elif next_state == 'StayAtState':
                state_machine.change_state(StayAtState())  
            elif next_state == 'Eight2State':
                state_machine.change_state(Eight2State())  
            elif next_state == 'OvalState':
                state_machine.change_state(OvalState())  
            elif next_state == 'ScanState':
                state_machine.change_state(ScanState())  

    def check_kind_movement(self,vec):
        """
            This method checks the kind of movement to be porformed for the current waypoint
        """
        if vec.x == 0 and vec.y != 0: 
            return 'stayat'
        else:
            return 'seek'

    def advance_waypoint(self,agent):
        """
            Advance to next waypoint
        """
        
        if self.sequence < len(self.waypoints) - 1:
            self.sequence+=1
            self.target = agent.get_position() + self.waypoints[self.sequence]
        else:
            self.sequence = 0  
            self.target = vec2(300, 60)
            self.finished = True

    def execute(self, agent):
        # logic to move drone to target
        pos = self.target - agent.get_position() 
        ang = atan2(pos.y,pos.x)

        # inciar sequencia
        way_point = self.waypoints[self.sequence]

        if self.check_kind_movement(way_point) == 'seek':
            agent.seek(self.target)
            if pos.length() < 5:
                # chegou
                self.advance_waypoint(agent)

        else:
            # perform rotation 
            # e : error of the angle accepted
            e = FORWARD_SPEED / self.radius
            agent.seek_around(self.target,self.radius)
            if -pi/2 - e < ang < -pi/2 + e:
                # terminou a rotaçao, sai em -pi/2
                self.advance_waypoint(agent)
        
class EightState(State):
    """
        Drone will perform a eight movement
        this state is not finished 
        ** USE Eight2State
    """ 
    def __init__(self):
        # initialization code
        self.state_name = 'Eight'
        self.theta = 0 #Variavel para contagem do tempo de execução 
        print('Eight')
        self.target = vec2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2)
        self.c = 0

    def check_transition(self, agent, state_machine):
        # logic to check and execute state transition

        # Transição Novo target do mouse
        if agent.get_target():
            self.target = agent.get_target()
            agent.set_target(None)

    def execute(self, agent):
        # logic to move drone to target
        pos = self.target - agent.get_position() 
        self.theta = atan2(pos.y,pos.x)
        #print(f'angulo : {self.theta }')
        e = FORWARD_SPEED / RADIUS_TARGET
        if   pi - e < self.theta < pi + e and self.c == 0: 
            self.c += 1
            self.target += vec2(RADIUS_TARGET*2,0)
            print(f'volta') 

        if self.c == 1 and  -SAMPLE_TIME < self.theta < SAMPLE_TIME  :
            self.target -= vec2(RADIUS_TARGET*2,0)
            self.c = 0

        agent.seek_around(self.target)
       