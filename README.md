# 🚁 Drone Swarm Simulator

A robust, open-source framework for simulating drone swarms in search-and-rescue operations and target acquisition scenarios. This simulator provides researchers, developers, and robotics enthusiasts with a flexible platform to experiment with various swarm coordination strategies, decision-making algorithms, and search patterns.

![Version](https://img.shields.io/badge/version-1.0.0-blue)
![Python](https://img.shields.io/badge/python-3.8%2B-brightgreen)
![License](https://img.shields.io/badge/license-MIT-green)

## 🎯 Features

- Realistic drone  movement patterns
- Configurable search environments with obstacles
- Pre-implemented search strategies
- State machine-based decision logic for autonomous drone behavior
- Visualization tools
- Extensible architecture for implementing custom algorithms

## 📋 Requirements

- **Python 3.8+** (latest stable version recommended)
- **IDE/Editor** (VS Code or any preferred code editor)

## 🔧 Installation

### Dependencies

Install all required dependencies with a single command:

```bash
pip install -r requirements.txt
```

Or install individual components:

- **Pygame** - For rendering and simulation visualization
  ```bash
  pip install pygame
  ```

- **NumPy** - For efficient numerical computations
  ```bash
  pip install numpy
  ```

## 🚀 Getting Started

1. Clone the repository:
   ```bash
   git clone https://github.com/luizgiacomossi/Search_Drone_Swarms.git
   cd Search_Drone_Swarms
   ```

2. Run the simulator:
   ```bash
   python main.py
   ```

## 🏗️ Project Structure

```
Search_Drone_Swarms/
│
├── 📄 main.py            # Entry point and main simulation loop
├── 📄 constants.py       # Simulation parameters and configuration
├── 📄 utils.py           # Helper functions and utilities
│
├── 📄 vehicle.py         # Drone physics and movement controllers
├── 📄 state_machine.py   # Decision-making logic for drones
├── 📄 scan.py            # Search algorithms and patterns
├── 📄 grid.py            # Discrete environment representation
├── 📄 obstacle.py        # Environmental obstacle generation
│
├── 📁 model/             # Visual assets and drone sprites
├── 📁 examples/          # Example scenarios and configurations
└── 📄 requirements.txt   # Project dependencies
```

## 🎮 Controls & Interface

- **Space:** Pause/Resume simulation
- **R:** Reset simulation
- **+/-:** Adjust simulation speed
- **ESC:** Exit simulator
- **Mouse Click:** Place target/obstacle

## 🔍 Search Strategies

The simulator includes several pre-implemented search strategies:

- **Grid Search** - Systematic coverage of the entire area
- **Spiral Search** - Outward spiraling pattern from central point -> Not implemented yet
- **Random Walk** - Stochastic movement with collision avoidance -> Not implemented yet
- **Levy Flight** - Bio-inspired search pattern with occasional long jumps -> Not implemented yet

## 🔧 Configuration

Edit `constants.py` to modify simulation parameters:

```python
# Simulation parameters
SIM_WIDTH = 1200         # Width of simulation window
SIM_HEIGHT = 800         # Height of simulation window
NUM_DRONES = 10          # Number of drones in swarm
OBSTACLE_DENSITY = 0.05  # Density of obstacles (0.0 to 1.0)

# Drone parameters
DRONE_SPEED = 3.0        # Maximum drone speed
SENSOR_RANGE = 100       # Drone sensor detection radius
COMMUNICATION_RANGE = 200  # Inter-drone communication range
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📜 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 📚 Citation

If you use this simulator in your research, please cite:

```
@software{drone_swarm_simulator,
  author = {Luiz Giacomossi},
  title = {Drone Swarm Simulator},
  year = {2025},
  url = {https://github.com/luizgiacomossi/Search_Drone_Swarms}
}
```
