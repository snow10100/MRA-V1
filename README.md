# MRA-V1: 6-DOF Robotic Arm Project

A comprehensive robotic arm project integrating **ROS2 Control**, **MoveIt2**, **Gazebo simulation**, and **Arduino hardware control**. This project supports both real hardware operation and physics-based simulation for development and testing.

![ROS2 Kilted](https://img.shields.io/badge/ROS2-Kilted-blue)
![Gazebo Garden](https://img.shields.io/badge/GZ-Garden-green)
![Arduino](https://img.shields.io/badge/Arduino-Compatible-red)
![License](https://img.shields.io/badge/License-MIT-yellow)

## 🚀 Quick Start

### Prerequisites
- **Docker** and **Docker Compose** (recommended)
- **Ubuntu 24.04 LTS** (for native installation)
- **ROS2 Kilted Kaiju** + **Gazebo Garden (GZ)** (for native installation)

### Launch Commands

```bash
# 🔨 Build First (recommended)
docker compose run build                  # Build all ROS2 packages

# 🎮 Simulation Mode
docker compose run sim                    # Gazebo simulation only
docker compose run moveit-sim             # MoveIt + Gazebo simulation

# 🤖 Hardware Mode  
docker compose run hardware               # Hardware control only
docker compose run moveit-hardware        # MoveIt + hardware control

# 🛠️ Development
docker compose run dev                    # Development shell (CPU)
docker compose run gpu-dev                # Development shell (GPU)
```

## 📁 Project Structure

```
MRA-V1/
├── arm_6dof/                    # Hardware interface package
├── arm_desc/                    # Robot URDF description
├── arm_moveit_config/           # MoveIt2 configuration
├── robot_v3/                    # Arduino firmware (latest)
├── worlds/                      # Gazebo world files
├── cont.py                      # Manual text controller
├── cont_ds4.py                  # PS4 controller interface
└── docker-compose.yml           # Docker configuration
```

## 🏗️ Architecture

### Dual Operation Modes

#### 🎯 Simulation Mode (GZ/Gazebo)
- **Physics Engine**: Gazebo Garden (GZ) with realistic joint dynamics
- **Workspace Environment**: Pre-configured world with manipulatable objects
- **Path Planning**: Collision detection with accurate meshes
- **Control Flow**: `GZ Sim → ROS2 Control → MoveIt2`

#### ⚙️ Hardware Mode (Arduino)
- **Hardware**: Custom 3D printed 6-DOF robotic arm
- **Control**: 6 stepper motors + gripper servo + mobile base
- **Communication**: Serial protocol (`/dev/ttyACM0`, 9600 baud)
- **Control Flow**: `Arduino → Serial → ROS2 Control → MoveIt2`

### ROS2 Packages

| Package | Description |
|---------|-------------|
| **arm_6dof** | Hardware interface for Arduino communication |
| **arm_desc** | Robot URDF with STL meshes and Gazebo support |
| **arm_moveit_config** | MoveIt2 motion planning configuration |

## 🔧 Hardware Specifications

- **Joints**: 6-DOF with ±π radian limits (±7680 Arduino steps)
- **Motors**: Stepper motors with AccelStepper library
- **Additional**: Gripper servo + 2 drive wheel servos
- **Communication**: Serial commands (`M{1-6},{mode},{value}`, `G{angle}`)
- **Platform**: Arduino with custom firmware (robot_v3.ino)

## 🌍 Simulation Environment

The Gazebo workspace (`worlds/arm_workspace.world`) includes:
- **Work table** for realistic manipulation scenarios
- **Target objects**: Red box, blue cylinder, green sphere
- **Obstacles**: Yellow box for path planning challenges  
- **Container tray** for pick-and-place operations
- **Realistic lighting** and **physics simulation**

## 🛠️ Development Workflow

1. **🎮 Prototype in Simulation**: Test algorithms safely in Gazebo
2. **📋 Plan with MoveIt2**: Generate collision-free motion plans
3. **✅ Validate on Hardware**: Deploy tested configurations to real robot
4. **🔄 Iterate Rapidly**: Use simulation for fast development cycles

## 📚 Documentation

- **[CLAUDE.md](CLAUDE.md)**: Comprehensive technical documentation
- **Configuration Files**: See `arm_moveit_config/config/` directory
- **Launch Files**: Available in `arm_*/launch/` directories

## 🎮 Control Options

### Manual Control
```bash
python3 cont.py                  # Text-based controller with recording
python3 cont_ds4.py             # PlayStation DS4 controller
```

### Programmatic Control
- **MoveIt2**: Motion planning and execution
- **ROS2 Control**: Direct joint control
- **Serial Interface**: Low-level Arduino communication

## 🚀 Advanced Usage

### Custom Launch Modes
```bash
# Environment variables
docker compose run -e LAUNCH_MODE=gazebo sim
docker compose run -e LAUNCH_MODE=moveit-gazebo moveit-sim

# Interactive development
docker compose run dev bash
```

### Native Installation
```bash
# Build workspace
colcon build --packages-select arm_6dof arm_desc arm_moveit_config
source install/setup.bash

# Launch simulation
ros2 launch arm_moveit_config gazebo_demo.launch.py

# Launch hardware
ros2 launch arm_moveit_config demo_real.launch.py
```

## 🤝 Contributing

1. **Development Environment**: Use `docker compose run dev`
2. **Testing**: Validate in simulation before hardware deployment
3. **Code Style**: Follow ROS2 and Python conventions
4. **Safety**: Always test motion plans in simulation first

## 📄 License

This project is licensed under the MIT License - see individual package licenses for details.

## 🆘 Support

For issues and questions:
- Check [CLAUDE.md](CLAUDE.md) for technical details
- Review configuration files in `arm_moveit_config/config/`
- Test in simulation mode first for debugging

---

**🎯 Ready to start?** Run `docker compose run moveit-sim` and explore the robotic arm in Gazebo simulation!