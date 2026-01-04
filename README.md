# ROS2 Autonomous Mobile Robot - SLAM & Navigation

A comprehensive ROS2-based autonomous mobile robot system featuring SLAM-based mapping, LiDAR perception, and Nav2 stack integration for robust indoor navigation in simulation environments.

## 📋 Overview

This project implements a fully autonomous mobile robot capable of navigating complex indoor environments using simultaneous localization and mapping (SLAM), real-time obstacle avoidance, and adaptive path planning. The system is designed for simulation-first development with Gazebo and RViz integration, enabling rapid prototyping and parameter tuning before hardware deployment.

## ✨ Key Features

- **SLAM-Based Mapping & Localization**: Real-time map building and robot pose estimation using LiDAR sensor data
- **Nav2 Stack Integration**: Complete navigation pipeline including global planning, local obstacle avoidance, and recovery behaviors
- **LiDAR Perception**: 2D/3D point cloud processing for environmental awareness and obstacle detection
- **Custom Robot Models**: URDF/Xacro-based robot descriptions with accurate kinematics and sensor configurations
- **Simulation Environment**: Gazebo-based testing environments with customizable worlds and obstacles
- **Real-Time Visualization**: RViz integration for monitoring robot state, sensor data, and navigation performance
- **Tuned Navigation Parameters**: Optimized planner, controller, costmap, and PID parameters for smooth trajectories and reliable goal convergence

## 🏗️ System Architecture

The system consists of several interconnected components:

- **Perception Layer**: LiDAR sensor processing and obstacle detection
- **Localization Layer**: SLAM algorithms (SLAM Toolbox/Cartographer) for pose estimation
- **Planning Layer**: Nav2 global planner for optimal path generation
- **Control Layer**: Nav2 local planner/controller for trajectory tracking and obstacle avoidance
- **Recovery Layer**: Automated recovery behaviors for handling navigation failures
- **Simulation Layer**: Gazebo physics simulation and RViz visualization

## 🔧 Prerequisites

- **ROS2 Distribution**: Humble Hawksbill (or newer)
- **Operating System**: Ubuntu 22.04 LTS
- **Dependencies**:
  - Nav2 Stack
  - SLAM Toolbox or Cartographer
  - Gazebo Classic or Ignition Gazebo
  - RViz2
  - robot_state_publisher
  - joint_state_publisher

## 📦 Installation

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/yourusername/autonomous-mobile-robot.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
source ~/ros2_ws/install/setup.bash
```

## 🚀 Usage

### 1. Launch Gazebo Simulation

```bash
ros2 launch autonomous_mobile_robot gazebo.launch.py
```

This starts the Gazebo simulator with your custom robot model and world environment.

### 2. Start SLAM Mapping

```bash
ros2 launch <your_package> slam.launch.py
```

Launches SLAM Toolbox for real-time mapping and localization.

### 3. Launch Navigation Stack

```bash
ros2 launch <your_package> navigation.launch.py
```

Starts the Nav2 stack with global planner, local controller, and recovery behaviors.

### 4. Visualize in RViz

```bash
ros2 launch <your_package> rviz.launch.py
```

Opens RViz with pre-configured visualization settings for robot state, laser scans, maps, and navigation paths.

### 5. Set Navigation Goals

Use RViz's "2D Goal Pose" tool to set navigation targets, or programmatically send goals:

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}"
```

## ⚙️ Configuration

### Robot Model (URDF/Xacro)

Robot description files are located in `urdf/` directory. Key components include:
- Base footprint and chassis
- Wheel assemblies with accurate kinematics
- LiDAR sensor with appropriate placement and FOV
- Accurate mass, inertia, and collision properties

### Navigation Parameters

Navigation configuration files are in `config/` directory:

- `nav2_params.yaml`: Global/local planner, controller, and costmap settings
- `slam_params.yaml`: SLAM algorithm configuration
- `rviz_config.rviz`: Visualization preferences

### Tuned Parameters

The following parameters have been optimized for improved performance:

**Global Planner**:
- Path smoothing algorithms
- Inflation radius for safe clearance
- Path resolution and tolerance

**Local Controller (DWB/TEB)**:
- Trajectory generation parameters
- Linear and angular velocity limits
- Obstacle margin and clearance
- Goal tolerance thresholds

**Costmaps**:
- Inflation layer parameters
- Obstacle layer update frequency
- Rolling window size
- Static/dynamic obstacle handling

**PID Controllers**:
- Proportional, Integral, Derivative gains
- Velocity feedback tuning
- Response time optimization

## 📊 Performance Characteristics

- **Trajectory Smoothness**: Minimized oscillations and jerky movements through controller tuning
- **Goal Convergence**: Reliable arrival at target poses with appropriate tolerance settings
- **Obstacle Avoidance**: Real-time dynamic obstacle detection and avoidance
- **Recovery Behaviors**: Automated handling of stuck situations (rotation, backup, etc.)
- **Sensor Noise Robustness**: Filtered sensor data and appropriate costmap parameters to handle noisy LiDAR readings

## 📁 Project Structure

```
.
├── config/
│   ├── nav2_params.yaml
│   ├── slam_params.yaml
│   └── rviz_config.rviz
├── launch/
│   ├── gazebo.launch.py
│   ├── slam.launch.py
│   ├── navigation.launch.py
│   └── rviz.launch.py
├── urdf/
│   ├── robot.urdf.xacro
│   ├── robot_base.xacro
│   └── sensors.xacro
├── worlds/
│   └── test_world.world
├── maps/
│   └── saved_maps/
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 🔍 Troubleshooting

### Robot doesn't move
- Check if all required nodes are running: `ros2 node list`
- Verify velocity commands: `ros2 topic echo /cmd_vel`
- Ensure costmaps are properly configured

### SLAM not working
- Verify LiDAR data: `ros2 topic echo /scan`
- Check transform tree: `ros2 run tf2_tools view_frames`
- Adjust SLAM parameters for your environment

### Navigation fails frequently
- Review costmap parameters and inflation radius
- Check velocity limits and acceleration constraints
- Increase obstacle clearance margins
- Review recovery behavior configurations

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## 📄 License

This project is licensed under the [MIT License](LICENSE).

## 📧 Contact

For questions or support, please open an issue in the repository.

## 🙏 Acknowledgments

- ROS2 Navigation Team for the Nav2 stack
- SLAM Toolbox maintainers
- Open Robotics for Gazebo simulator
- ROS community for extensive documentation and support

---

**Note**: Replace `<your_package>` and `<your-repository-url>` with your actual package name and repository URL throughout this README.
