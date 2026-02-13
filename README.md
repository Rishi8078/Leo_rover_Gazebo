# Gazebo Simulation Project

This project simulates a Leo Rover in a simple Sand world using Gazebo Sim.

## Prerequisites

Ensure you have ROS 2 installed (Humble or Jazzy recommended).
The following packages are required:

- `ros_gz_sim`
- `leo_description`
- `ros_gz_bridge`
- `robot_state_publisher`
- `xacro`

## How to Run

1. Open a terminal in this directory.
2. Run the launch script:

```bash
ros2 launch ./launch_leo.py
```

This will:
- Start Gazebo Sim with the `simple_world.sdf`
- Spawn the Leo Rover model
- Start the ROS-Gazebo bridge
- Publish robot state

## Custom Spawn Position

You can set the initial X, Y, and Z coordinates using launch arguments:

```bash
ros2 launch ./launch_leo.py z:=1.0 x:=2.0
```

- `x`: Spawn X position (default: 0.0)
- `y`: Spawn Y position (default: 0.0)
- `z`: Spawn Z position (default: 0.5)

## Troubleshooting

- If you see warnings about mesh normals for `sand_terrain.dae`, this is a known issue but the simulation should still run.
- Ensure you have sourced your ROS 2 environment (`source /opt/ros/humble/setup.bash`).
