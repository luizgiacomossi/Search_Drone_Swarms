import time
import csv
import matplotlib.pyplot as plt
from constants import SAVE_RESULTS

class ExperimentManager(object):
    def __init__(self, in_repetitions, in_num_swarm, in_num_obstacles, in_algorithms):
        self.current_repetition = 0
        self.in_num_swarm = []
        self.in_num_obstacles = []
        self.in_algorithms = []
        
        # Inputs of Rate
        self.in_repetitions = in_repetitions * len(in_num_swarm) * len(in_num_obstacles) * len(in_algorithms)
        
        res = [[i, j, k] for i in enumerate(in_num_swarm) 
                         for j in enumerate(in_num_obstacles) 
                         for k in enumerate(in_algorithms) ]

        for r in enumerate(res):
            print(str(r))
            self.in_num_swarm += [r[1][0][1]] * in_repetitions 
            self.in_num_obstacles += [r[1][1][1]] * in_repetitions 
            self.in_algorithms += [r[1][2][1]] * in_repetitions 
        
        # Outputs of Rate
        self.out_time_mission = []
        self.out_time_target = []
        self.out_num_uav = []
        self.print_plan_rate()

    def set_time_target(self, time_target):
        self.out_time_target.append(time_target)

    def set_out(self, out_time_mission, out_num_uav):
        self.out_time_mission.append(out_time_mission)
        self.out_num_uav.append(out_num_uav)

    def next_simulation(self):
        if self.in_repetitions - 1 == self.current_repetition:
            return False
        else:
            self.current_repetition = self.current_repetition + 1
            self.print_simulation()
            return True

    def print_plan_rate(self):
        for idx in range(0, self.in_repetitions):
            print(f'{idx+1} - num_obstacles: {self.in_num_obstacles[idx]}, num_swarm : {self.in_num_swarm[idx]}, algorithm : {self.in_algorithms[idx].to_string()},')

    def print_simulation(self):
        return f'{self.current_repetition+1} - num_swarm: {self.in_num_swarm[self.current_repetition]}, num_obstacles: {self.in_num_obstacles[self.current_repetition]}, Algorithm: {self.in_algorithms[self.current_repetition].to_string()}'

    def print_simulation_idx(self, idx):
        # Note: 'time' variable was used in original code but not defined in method scope. 
        # Assuming it refers to mission time which is in out_time_mission
        mission_time = self.out_time_mission[idx] if idx < len(self.out_time_mission) else 0.0
        return f'{idx+1} - Time: {mission_time:.2f}, num_uav: {self.out_num_uav[idx]}, num_swarm: {self.in_num_swarm[idx]}, num_obstacles: {self.in_num_obstacles[idx]}'

    def print_rate(self):
        num_d = []
        time_sim = []
        time_target = []
        num_obst = []
        qntd_drones = []
        idx_sim = []

        for idx in range(0, len(self.out_time_target)):
            print(f'{idx+1} - Time Target: {self.out_time_target[idx]}, Time Mission: {self.out_time_mission[idx]}, num_uav: {self.out_num_uav[idx]}, num_swarm: {self.in_num_swarm[idx]}, num_obstacles: {self.in_num_obstacles[idx]}')
            num_d.append(self.in_num_swarm[idx])
            idx_sim.append(idx+1)
            time_target.append(self.out_time_target[idx])
            time_sim.append(self.out_time_mission[idx])
            qntd_drones.append(self.out_num_uav[idx])
            num_obst.append(self.in_num_obstacles[idx])

        plt.xlabel('Idx')
        plt.ylabel('Time')
        plt.title('History of simulations')
        #plt.plot(idx_sim, time_sim, 'r--', idx_sim,qntd_drones, 'g^' , idx_sim,num_obst,'bs' )
        plt.plot(idx_sim,qntd_drones, 'g^', idx_sim, num_d  )
        plt.show()
        #plt.plot(t, t, 'r--', t, t**2, 'bs', t, t**3, 'g^') 

    def save_csv(self):
        if not SAVE_RESULTS:
            return
            
        with open('result.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Execution", "N Drones", "N Obstacles", "Algorithm", "Time Found Target", "Time Mission Completed"])
            print(len(self.out_time_mission))
            for idx in range(0, self.in_repetitions):
                #writer.writerow([idx+1, self.in_num_swarm[idx], self.in_num_obstacles[idx], self.in_algorithms[idx], self.out_time_target[idx], self.out_time_mission[idx]])
                writer.writerow([idx+1, self.in_num_swarm[idx], self.in_num_obstacles[idx], self.in_algorithms[idx].to_string(), self.out_time_target[idx], self.out_time_mission[idx]])
