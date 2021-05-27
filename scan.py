from constants import *

class ScanInterface:
    def to_string(self) -> str:
        """return the name of Algorithm"""
        pass

    def scan(self, simulation, list_obst):
        """run the scan algorithm"""
        pass

class DefineTargetScan(ScanInterface):
    def to_string(self) -> str:
        return 'DefineTargetScan'

    def scan(self, simulation, list_obst):
        index = 0 # index is used to track current drone in the simulation list
        for _ in simulation.swarm:
            # checks if drones colided with eachother

            ## collision avoindance is not implemented yet
            _.collision_avoidance(simulation.swarm, index)
            _.check_collision(simulation.swarm,list_obst,index) 
            _.update()
            _.draw(simulation.screenSimulation.screen) 
            # index to keep track of  drone in the list
            index += 1
            # writes drone id
            img = simulation.screenSimulation.font20.render(f'Drone {index}', True, LIGHT_BLUE)
            simulation.screenSimulation.screen.blit(img, _.get_position()+(0,20))
            # writes drone current behavior
            #img = simulation.screenSimulation.font20.render(_.behavior.get_current_state(), True, BLUE)
            #simulation.screenSimulation.screen.blit(img, _.get_position()+(0,30))
            # writes drone current position in column and row
            p = _.get_position()
            col = int(p.x/RESOLUTION) + 1
            row = int(p.y/RESOLUTION) + 1
            #img = simulation.screenSimulation.font20.render(f'Pos:{col},{row}', True, BLUE)
            #simulation.screenSimulation.screen.blit(img, _.get_position()+(0,40))
            
            if _.reached_goal(simulation.target_simulation):
                print(f"Drone {index} atingiu o target")

class RandoWalkScan(ScanInterface):
    def to_string(self) -> str:
        return 'RandoWalkScan'

    def scan(self, simulation, list_obst):
        pass

class SnookerScan(ScanInterface):
    def to_string(self) -> str:
        return 'SnookerScan'

    def scan(self, simulation, list_obst):
        pass

class MeshScan(ScanInterface):
    def to_string(self) -> str:
        return 'MeshScan'

    def scan(self, simulation, list_obst):
        pass