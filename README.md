# TurtleBot3 Maze Workspace

A ROS 2 workspace scaffold for TurtleBot3 maze simulation, navigation, and Deep Q-Network (DQN) experiments.

## Current status

This project is under development. `maze_world` has an `ament_cmake` package definition and resource installation rules. The maze world, launch files, and DQN Python modules are currently empty. The `maze_navigation` and `maze_dqn` package definitions are also placeholders, so a full workspace build and end-to-end simulation are not ready yet.

## Workspace structure

```text
src/
├── maze_world/                  # Simulation resources
│   ├── launch/simulation.launch.py
│   └── worlds/maze.sdf
├── maze_navigation/             # Navigation and mapping placeholders
│   └── launch/
│       ├── nav2.launch.py
│       └── slam.launch.py
└── maze_dqn/                    # Reinforcement learning placeholders
    └── maze_dqn/
        ├── agent.py
        ├── train.py
        └── evaluate.py
```

`build/`, `install/`, and `log/` contain generated colcon outputs and are excluded by `.gitignore`.

## Prerequisites

- A ROS 2 installation with `ament_cmake`.
- Python 3 and pip.
- The `colcon` build tool and `rosdep` for ROS dependencies.

The project does not yet specify a ROS 2 distribution or simulator version. TurtleBot3, simulation, navigation, and machine learning dependencies will need to be declared as those features are implemented.

## Setup

Run these commands from the workspace root. Replace `<ros-distro>` with your installed ROS 2 distribution.

```bash
source /opt/ros/<ros-distro>/setup.bash
python3 -m pip install -r requirements.txt
```

`requirements.txt` currently contains no dependencies because the Python modules have not been implemented. ROS dependencies belong in each package's `package.xml`.

To install dependencies for the package that currently has a valid manifest, use an initialized `rosdep` installation:

```bash
rosdep install --from-paths src/maze_world --ignore-src -r -y
```

## Build

Build the currently defined package:

```bash
colcon build --symlink-install --packages-select maze_world
source install/setup.bash
```

After completing the other package manifests and build configuration, install dependencies and build the entire workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Planned functionality

- Define the maze and robot simulation in `maze_world`.
- Implement the simulation, SLAM, and Nav2 launch files.
- Implement the DQN agent, training loop, and evaluation workflow.
- Declare package dependencies and add runnable entry points.

Launch and training commands will be documented once these files are implemented.

## License

The `maze_world` package declares Apache-2.0; its license text is in [src/maze_world/LICENSE](src/maze_world/LICENSE). The remaining package placeholders do not yet declare a license.
