# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a 6-DOF robotic arm project that integrates ROS2 Control with both Arduino hardware and Gazebo simulation. The system consists of three ROS2 packages for hardware control, robot description, and MoveIt motion planning, plus Arduino firmware and Python controllers. The project supports both real hardware operation and Gazebo simulation for development and testing.

## System Requirements

- **ROS2**: Kilted Kaiju (latest stable LTS release)
- **Gazebo**: Gazebo Garden (GZ) - compatible with ROS2 Kilted
- **OS**: Ubuntu 24.04 LTS (recommended for ROS2 Kilted)
- **Hardware**: Custom 3D printed 6-DOF robotic arm with Arduino control

## Build Commands

```bash
# Build all ROS2 packages
colcon build

# Build with verbose output
colcon build --verbose

# Build specific package
colcon build --packages-select arm_6dof

# Source the workspace
source install/setup.bash
```

## Running the System

### Using Docker (Recommended)

#### Quick Launch Commands
```bash
# Build packages first (recommended)
docker compose run build                  # Build all ROS2 packages

# Development environments
docker compose run dev                    # Development shell (CPU)
docker compose run gpu-dev                # Development shell (GPU)

# Simulation modes
docker compose run sim                    # Gazebo simulation only
docker compose run moveit-sim             # MoveIt + Gazebo simulation

# Hardware modes  
docker compose run hardware               # Hardware control only
docker compose run moveit-hardware        # MoveIt + hardware control
```

#### Advanced Launch Options
```bash
# Custom environment variables
docker compose run -e LAUNCH_MODE=gazebo sim
docker compose run -e LAUNCH_MODE=moveit-gazebo moveit-sim

# Interactive development
docker compose run dev bash               # Start development shell
docker compose run gpu-dev bash          # Start GPU development shell
```

### Hardware Mode
```bash
# Basic robot visualization in RViz with real hardware
ros2 launch arm_desc display.launch.py

# MoveIt demo with real hardware interface
ros2 launch arm_moveit_config demo_real.launch.py

# Start hardware interface and controllers for real robot
ros2 launch arm_moveit_config move_group.launch.py
```

### Simulation Mode (Gazebo)
```bash
# Launch Gazebo simulation with robot model
ros2 launch arm_desc gazebo.launch.py

# MoveIt demo with Gazebo simulation
ros2 launch arm_moveit_config demo.launch.py

# Combined Gazebo + MoveIt launch (recommended)
ros2 launch arm_moveit_config gazebo_demo.launch.py
```

### Visualization Only
```bash
# Basic robot visualization in RViz (no hardware/simulation)
ros2 launch arm_desc display.launch.py
```

### Manual Control
```bash
# Text-based controller with recording/playback
python3 cont.py

# PlayStation DS4 controller
python3 cont_ds4.py
```

## Architecture

### ROS2 Packages Structure
- **arm_6dof**: Hardware interface package that communicates with Arduino over serial (`/dev/ttyACM0` at 9600 baud)
- **arm_desc**: Robot URDF description with 5 revolute joints, STL meshes, and Gazebo simulation support
- **arm_moveit_config**: MoveIt2 configuration for motion planning and execution in both hardware and simulation modes

### Dual Mode Operation

#### Hardware Mode
- 6 stepper motors controlled via Arduino (robot_v3.ino firmware)
- 1 gripper servo + 2 drive wheel servos for mobile base
- Serial command protocol: `M{1-6},{mode},{value}` for motor control, `G{angle}` for gripper
- Control Flow: Arduino firmware ← Serial ← arm_6dof hardware interface ← ROS2 Control ← MoveIt2

#### Simulation Mode (Gazebo)
- Physics simulation with accurate joint dynamics
- Visual and collision meshes for realistic simulation
- Gazebo plugins for sensor simulation and physics interaction
- Control Flow: Gazebo physics engine ← Gazebo ROS2 Control ← MoveIt2

### Configuration Switching
- **Hardware Config**: `robotic_arm_real.ros2_control.xacro` - Real hardware interface
- **Simulation Config**: `robotic_arm.ros2_control.xacro` - Gazebo simulation interface
- **URDF Variants**: Separate URDF configurations for hardware vs simulation needs

## Key Configuration Files

### Hardware Configuration
- `arm_moveit_config/config/robotic_arm_real.ros2_control.xacro`: Real hardware interface configuration
- `arm_moveit_config/config/robotic_arm_real.urdf.xacro`: Hardware-specific URDF variant
- `arm_moveit_config/launch/demo_real.launch.py`: Hardware demo launch file
- `robot_v3/robot_v3.ino`: Arduino firmware (latest version)

### Simulation Configuration
- `arm_moveit_config/config/robotic_arm.ros2_control.xacro`: Gazebo simulation interface
- `arm_moveit_config/config/robotic_arm.urdf.xacro`: Simulation-optimized URDF
- `arm_desc/launch/gazebo.launch.py`: Gazebo simulation launch file
- `arm_moveit_config/launch/gazebo_demo.launch.py`: Combined Gazebo + MoveIt launch
- `worlds/arm_workspace.world`: Gazebo world with workspace objects

### Shared Configuration
- `arm_moveit_config/config/ros2_controllers.yaml`: Controller parameters (both modes)
- `arm_desc/urdf/robot.urdf.xacro`: Base robot kinematic model
- `arm_moveit_config/config/robotic_arm.srdf`: MoveIt semantic description

## Development Notes

### Hardware Specifications
- Joint limits: ±7680 steps (±π radians) for all joints
- Arduino uses AccelStepper library for smooth motion
- Serial protocol supports relative/absolute positioning and state reading
- Communication: `/dev/ttyACM0` at 9600 baud

### Simulation Features
- **Physics Engine**: Gazebo Garden (GZ) with realistic joint dynamics
- **Collision Detection**: Accurate collision meshes for path planning
- **Sensor Simulation**: Support for camera, lidar, and other sensors
- **Plugin System**: Custom GZ plugins for specialized behaviors
- **Modern Interface**: Uses `gz sim` command instead of legacy `gazebo`
- **Workspace Environment**: Pre-configured world with work table and manipulatable objects:
  - Red box, blue cylinder, green sphere (target objects)
  - Yellow obstacle box for path planning challenges
  - Container tray for pick-and-place tasks
  - Realistic lighting and physics simulation

### Control Systems
- **MoveIt2**: Joint trajectory controller for coordinated arm movement
- **ROS2 Control**: Hardware abstraction layer supporting both real and simulated robots
- **Python Controllers**: Direct hardware access bypassing ROS2 Control (hardware only)

### Development Workflow
1. **Prototype in Simulation**: Test algorithms and motion plans in Gazebo
2. **Validate on Hardware**: Deploy tested configurations to real robot
3. **Iterate Safely**: Use simulation for rapid development and testing

## Quick Reference

### Most Common Commands
```bash
# Start simulation for development/testing
docker compose run moveit-sim

# Start hardware control for real robot
docker compose run moveit-hardware

# Development shell for building/debugging
docker compose run dev

# Manual control interfaces
python3 cont.py        # Text-based controller
python3 cont_ds4.py    # PlayStation DS4 controller
```

### Build and Development
```bash
# Inside development container
colcon build --packages-select arm_6dof arm_desc arm_moveit_config
source install/setup.bash

# Test specific package
colcon build --packages-select arm_6dof
colcon test --packages-select arm_6dof
```

### Troubleshooting
- **No Arduino device**: Check `/dev/ttyACM0` permissions and connection
- **Gazebo won't start**: Ensure GPU support with `docker compose run gpu-dev`
- **Build failures**: Clean workspace with `rm -rf build/ install/ log/`
- **MoveIt planning fails**: Check joint limits in `joint_limits.yaml`