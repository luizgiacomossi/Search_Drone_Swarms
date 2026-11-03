import random
from math import pi, atan2, inf
from constants import *
import numpy as np
import pygame 
import time

NOT_VISITED = 0      
VISITED = 1
OBSTACLE = 2
vec2 = pygame.math.Vector2


class FiniteStateMachine(object):
    """
    A robust finite state machine with lifecycle hooks (enter/exit).
    """
    def __init__(self, state):
        self.state = state
        self._current_agent = None

    def change_state(self, new_state, agent=None):
        target_agent = agent or self._current_agent

        if target_agent is not None and hasattr(self.state, 'exit'):
            self.state.exit(target_agent)

        self.state = new_state
        self.state._entered = False

        if target_agent is not None and hasattr(self.state, 'enter'):
            self.state.enter(target_agent)
            self.state._entered = True

    def update(self, agent):
        self._current_agent = agent

        # Initial lifecycle enter hook if not yet initialized
        if not getattr(self.state, '_entered', False):
            if hasattr(self.state, 'enter'):
                self.state.enter(agent)
            self.state._entered = True

        self.state.check_transition(agent, self)
        self.state.execute(agent)

    def get_current_state(self):
        return self.state.state_name


class State(object):
    """
    Abstract state class with lifecycle management.
    """
    def __init__(self, state_name):
        self.state_name = state_name
        self._entered = False

    def enter(self, agent):
        """Called when entering this state."""
        pass

    def exit(self, agent):
        """Called when exiting this state."""
        pass

    def check_transition(self, agent, fsm):
        """Checks conditions and executes state transition if needed."""
        raise NotImplementedError("Subclasses must implement check_transition")

    def execute(self, agent):
        """Executes the state behavior for the agent."""
        raise NotImplementedError("Subclasses must implement execute")


class SeekState(State):
    """
    Drone will seek a target location directly.
    """
    def __init__(self, target=None):
        super().__init__('SeekState')
        self.target = vec2(target) if target is not None else None
        self.time_executing = 0
        self.finished = False
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 3
        self.time_blocked = 0

    def enter(self, agent):
        if self.target is None:
            if agent.get_target():
                self.target = vec2(agent.get_target())
                agent.mission_target = vec2(agent.get_target())
            elif getattr(agent, 'mission_target', None) is not None:
                self.target = vec2(agent.mission_target)
            else:
                self.target = vec2(agent.get_position())

    def check_transition(self, agent, state_machine):
        # New target from user mouse click or re-assignment
        new_target = agent.get_target()
        if new_target is not None:
            self.target = vec2(new_target)
            agent.mission_target = vec2(new_target)
            agent.set_target(None)
            self.finished = False
            self.time_blocked = 0

        if self.target is None:
            self.target = vec2(agent.mission_target) if getattr(agent, 'mission_target', None) else vec2(agent.get_position())

        # Check if drone reached the target radius
        d = self.target.distance_to(agent.get_position())
        if d <= RADIUS_TARGET and d > 3:
            self.finished = True
            self.state_name = 'Done'

        # Check if drone is blocked by an obstacle
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 30 and not self.finished:
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 2:
                pos_in_grid = agent.position_in_grid
                if getattr(agent, 'grid_map', None) is not None:
                    agent.grid_map.change_state_cell(pos_in_grid, OBSTACLE)
                state_machine.change_state(GoToClosestDroneState(), agent)
        elif dist >= 30 and not self.finished:
            self.time_blocked = 0
            self.state_name = 'SeekState'

    def execute(self, agent):
        if self.target is None:
            self.target = vec2(agent.mission_target) if getattr(agent, 'mission_target', None) else vec2(agent.get_position())

        agent.arrive(self.target)
        self.time_executing += SAMPLE_TIME

        # Sample location periodically to monitor progress
        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = vec2(agent.get_position())


class GoToClosestDroneState(State):
    """
    Drone moves toward the closest neighbor in the swarm to clear an obstacle.
    """
    def __init__(self):
        super().__init__('GoToClosestDroneState')
        self.time_executing = 0
        self.finished = False
        self.target = None

    def enter(self, agent):
        self.target = agent.get_closest_drone()
        if self.target is None:
            self.finished = True

    def check_transition(self, agent, state_machine):
        # New target override from mouse click
        if agent.get_target():
            state_machine.change_state(SeekState(), agent)
            return

        # Timeout to prevent getting stuck
        if self.time_executing > 3:
            state_machine.change_state(RandomTargetState(), agent)
            return

        # Reached close neighbor waypoint
        if self.finished:
            state_machine.change_state(SeekState(), agent)

    def execute(self, agent):
        if self.target is None:
            self.target = agent.get_closest_drone()

        if self.target is not None:
            agent.arrive(self.target)
            self.time_executing += SAMPLE_TIME
            if (self.target - agent.location).length() < SIZE_DRONE * 2 * 1.4:
                self.finished = True
        else:
            self.finished = True


class RandomTargetState(State):
    """
    Drone picks a local random offset target to unblock as last resort.
    """
    def __init__(self):
        super().__init__('RandomTargetState')
        self.time_executing = 0
        self.finished = False
        self.target = None

    def enter(self, agent):
        base_pos = getattr(agent, 'mission_target', None) or agent.get_position()
        random_offset = vec2(random.uniform(-400, 400), random.uniform(-400, 400))
        self.target = base_pos + random_offset

    def check_transition(self, agent, state_machine):
        if agent.get_target():
            state_machine.change_state(SeekState(), agent)
            return

        if self.finished:
            state_machine.change_state(SeekState(), agent)

    def execute(self, agent):
        if self.target is None:
            base_pos = getattr(agent, 'mission_target', None) or agent.get_position()
            self.target = base_pos + vec2(random.uniform(-400, 400), random.uniform(-400, 400))

        agent.arrive(self.target)
        self.time_executing += SAMPLE_TIME

        if (self.target - agent.location).length() < 10 or self.time_executing > 3:
            self.finished = True


class SearchTargetState(State):
    """
    Drone systematically searches unexplored 8-connected grid cells.
    """
    def __init__(self):
        super().__init__('SearchTargetState')
        self.time_executing = 0
        self.finished = False
        self.target = None
        self.waypoints = []
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 2
        self.time_blocked = 0

    def enter(self, agent):
        new_target = agent.get_target()
        if new_target is not None:
            self.target = vec2(new_target)
            agent.mission_target = vec2(new_target)
        elif getattr(agent, 'mission_target', None) is not None:
            self.target = vec2(agent.mission_target)
        else:
            self.target = vec2(agent.get_position())

        if getattr(agent, 'grid_map', None) is not None:
            self.waypoints = agent.grid_map.get_sucessors(agent.position_in_grid)

    def check_transition(self, agent, state_machine):
        # New target override from mouse click
        if agent.get_target():
            self.target = vec2(agent.get_target())
            agent.mission_target = vec2(agent.get_target())
            agent.set_target(None)

        if self.finished:
            state_machine.change_state(SeekState(), agent)
            return

        # Check if drone is blocked
        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 30 and not self.finished:
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 2:
                self.time_blocked = 0
                if getattr(agent, 'grid_map', None) is not None:
                    unvisited = agent.grid_map.get_cell_not_visited()
                    if unvisited is not None:
                        center = agent.grid_map.get_cell_center(unvisited)
                        self.target = vec2(center[0], center[1])
                    else:
                        self.target = vec2(random.uniform(100, WORLD_WIDTH - 100), random.uniform(100, WORLD_HEIGHT - 100))
                else:
                    self.target = vec2(random.uniform(100, WORLD_WIDTH - 100), random.uniform(100, WORLD_HEIGHT - 100))
        elif dist >= 30 and not self.finished:
            self.time_blocked = 0
            self.state_name = 'SearchTargetState'

    def execute(self, agent):
        if self.target is None:
            self.target = vec2(agent.mission_target) if getattr(agent, 'mission_target', None) else vec2(agent.get_position())

        agent.arrive(self.target)
        self.time_executing += SAMPLE_TIME

        if (self.target - agent.location).length() < 30:
            if getattr(agent, 'grid_map', None) is not None:
                self.waypoints = agent.grid_map.get_sucessors(agent.position_in_grid)
                if len(self.waypoints) > 0:
                    targ = random.choice(self.waypoints)
                    self.target = vec2(targ[0], targ[1])
                else:
                    unvisited = agent.grid_map.get_cell_not_visited()
                    if unvisited is not None:
                        center = agent.grid_map.get_cell_center(unvisited)
                        self.target = vec2(center[0], center[1])
                    else:
                        self.target = vec2(random.uniform(100, WORLD_WIDTH - 100), random.uniform(100, WORLD_HEIGHT - 100))
            else:
                self.target = vec2(random.uniform(100, WORLD_WIDTH - 100), random.uniform(100, WORLD_HEIGHT - 100))

        # Check if target is found by any drone in the swarm
        if getattr(agent, 'found', False):
            self.finished = True

        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = vec2(agent.get_position())


class RandomSearchState(State):
    """
    Drone performs random spatial search.
    """
    def __init__(self):
        super().__init__('RandomSearchState')
        self.time_executing = 0
        self.finished = False
        self.target = None
        self.memory_last_position = vec2(inf, inf)
        self.sampling_time = 2
        self.time_blocked = 0

    def enter(self, agent):
        self.target = getattr(agent, 'mission_target', None) or agent.get_position()

    def check_transition(self, agent, state_machine):
        if self.finished:
            state_machine.change_state(SeekState(), agent)
            return

        dist = self.memory_last_position.distance_to(agent.get_position())
        if dist < 70 and not self.finished:
            self.time_blocked += SAMPLE_TIME
            self.state_name = f'Blocked: {self.time_blocked:.2f}'
            if self.time_blocked > 1:
                self.time_blocked = 0
                self.target = vec2(random.uniform(100, WORLD_WIDTH - 100), random.uniform(100, WORLD_HEIGHT - 100))
        elif dist >= 70 and not self.finished:
            self.time_blocked = 0
            self.state_name = 'RandomSearchState'

    def execute(self, agent):
        if self.target is None:
            self.target = getattr(agent, 'mission_target', None) or agent.get_position()

        agent.arrive(self.target)
        self.time_executing += SAMPLE_TIME

        if (self.target - agent.location).length() < RADIUS_OBSTACLES * 2:
            self.target = vec2(random.uniform(100, WORLD_WIDTH - 100), random.uniform(100, WORLD_HEIGHT - 100))

        if getattr(agent, 'found', False):
            self.finished = True

        if self.time_executing >= self.sampling_time:
            self.time_executing = 0
            self.memory_last_position = vec2(agent.get_position())


def get_random_state():
    """
    Sample an instance of a search state.
    """
    states = [SeekState, SearchTargetState, RandomSearchState]
    chosen = random.choice(states)
    return chosen()
