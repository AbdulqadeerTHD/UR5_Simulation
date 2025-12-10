# UR5 Simulation - Complete Setup Guide

This repository contains a complete ROS2 Humble simulation setup for the UR5 robot with Robotiq 85 gripper in Gazebo. It includes three different simulation environments and mission scripts.

## 🤖 Robot Configuration

- **Robot**: Universal Robots UR5
- **Gripper**: Robotiq 85
- **Simulator**: Gazebo
- **ROS Version**: ROS2 Humble
- **Framework**: MoveIt2 (optional, scripts work without it)

---

## 📦 Installation

### Prerequisites

```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop -y

# Install required packages
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-trajectory-controller \
    python3-colcon-common-extensions \
    python3-vcstool
```

### Build the Workspace

```bash
cd ~/ur5_sm_ws
colcon build
source install/setup.bash
```

---

## 🎮 Three Simulation Environments

### 1. Simple Robot (Robot Only)

**Description**: Basic UR5 robot with camera and gripper in an empty Gazebo world.

**Launch Command**:
```bash
cd ~/ur5_sm_ws
source install/setup.bash
ros2 launch ur_yt_sim robot_only.launch.py
```

**What it includes**:
- UR5 robot arm
- Robotiq 85 gripper
- Camera sensor
- Empty Gazebo world

**Use cases**:
- Testing robot basic movements
- Learning robot kinematics
- Testing gripper control
- Camera visualization

---

### 2. Peg-in-Hole Mission

**Description**: Complete peg-in-hole assembly task with UR5 robot, peg, and hole block on an elevated platform.

**Launch Command**:
```bash
cd ~/ur5_sm_ws
source install/setup.bash
ros2 launch ur_yt_sim peg_hole.launch.py
```

**Environment Setup**:
- **Platform**: 0.6m × 0.4m × 0.02m at Z=0.3m
- **Block with Hole**: 
  - Position: X=0.6m, Y=0.2m, Z=0.3m
  - Dimensions: 10cm × 10cm × 5cm
  - Hole: 11mm radius, through-hole
- **Red Peg**: 
  - Position: X=0.6m, Y=0.35m, Z=0.33m
  - Dimensions: 20mm diameter, 6cm length
  - Orientation: Horizontal along X-axis

**Mission Script**:
```bash
# After launching the simulation, run the mission:
cd ~/ur5_sm_ws
source install/setup.bash
python3 src/ur_yt_sim/src/mc_vilet_peg_hole_mission.py
```

**Mission Sequence**:
1. Approach peg horizontally
2. Grasp peg with gripper
3. Lift peg
4. Rotate peg to vertical orientation
5. Move above hole
6. Insert peg into hole
7. Release peg
8. Retreat and return to home

**Setup Script (Place peg in gripper)**:
```bash
# If you want to start with peg already in gripper:
cd ~/ur5_sm_ws
source install/setup.bash
python3 src/ur_yt_sim/src/setup_peg_in_gripper.py
```

**Manual Control**:
```bash
# Move robot to home position
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "
{
  joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
  points: [{
    positions: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
    time_from_start: {sec: 2}
  }]
}"

# Open gripper
ros2 action send_goal /gripper_position_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 50.0}}"

# Close gripper
ros2 action send_goal /gripper_position_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.79, max_effort: 50.0}}"
```

---

### 3. Hammering Mission

**Description**: UR5 robot performing a hammering task with a hammer tool.

**Launch Command**:
```bash
cd ~/ur5_sm_ws
source install/setup.bash
ros2 launch ur_yt_sim hammering.launch.py
```

**Environment Setup**:
- UR5 robot with gripper
- Hammer tool model
- Work surface for hammering

**Use cases**:
- Tool manipulation
- Repetitive motion tasks
- Force control applications

---

## 🚀 Quick Start Guide

### Terminal Setup (Recommended: 3 Terminals)

**Terminal 1 - Gazebo Simulation**:
```bash
cd ~/ur5_sm_ws
source install/setup.bash
ros2 launch ur_yt_sim peg_hole.launch.py
```

**Terminal 2 - MoveIt (Optional, for visualization)**:
```bash
cd ~/ur5_sm_ws
source install/setup.bash
ros2 launch ur5_camera_gripper_moveit_config move_group.launch.py
```

**Terminal 3 - Mission Script**:
```bash
cd ~/ur5_sm_ws
source install/setup.bash
python3 src/ur_yt_sim/src/mc_vilet_peg_hole_mission.py
```

---

## 📁 Project Structure

```
ur5_sm_ws/
├── src/
│   ├── ur_yt_sim/              # Main simulation package
│   │   ├── launch/             # Launch files
│   │   │   ├── robot_only.launch.py
│   │   │   ├── peg_hole.launch.py
│   │   │   └── hammering.launch.py
│   │   ├── worlds/              # Gazebo world files
│   │   │   ├── robot_only.world
│   │   │   ├── peg_hole.world
│   │   │   └── hammering.world
│   │   ├── models/              # Gazebo models
│   │   │   ├── peg_red/
│   │   │   ├── peg_hole/
│   │   │   └── hammer_tool/
│   │   └── src/                 # Python scripts
│   │       ├── mc_vilet_peg_hole_mission.py
│   │       └── setup_peg_in_gripper.py
│   ├── ur5_camera_gripper_moveit_config/  # MoveIt configuration
│   └── ...                      # Other dependencies
├── build/                       # Build directory
├── install/                     # Install directory
└── log/                         # Log directory
```

---

## 🎯 Key Features

### Peg-in-Hole Mission Features:
- ✅ Geometric inverse kinematics (no MoveIt dependency)
- ✅ Sequential mission execution
- ✅ Gripper control integration
- ✅ Error handling and recovery
- ✅ Detailed logging

### Robot Capabilities:
- 6-DOF UR5 manipulator
- Robotiq 85 gripper (2-finger)
- Camera sensor
- Joint trajectory control
- MoveIt2 integration (optional)

---

## 🔧 Troubleshooting

### Issue: Gazebo doesn't start
```bash
# Kill existing Gazebo processes
killall gzserver gzclient
# Restart
ros2 launch ur_yt_sim peg_hole.launch.py
```

### Issue: Robot not moving
```bash
# Check if controllers are running
ros2 topic list | grep joint
# Check joint states
ros2 topic echo /joint_states
```

### Issue: Gripper not responding
```bash
# Check gripper action server
ros2 action list
# Should see: /gripper_position_controller/gripper_cmd
```

### Issue: Build errors
```bash
# Clean and rebuild
cd ~/ur5_sm_ws
rm -rf build install log
colcon build
```

---

## 📝 Mission Scripts

### `mc_vilet_peg_hole_mission.py`
Complete peg-in-hole mission automation script. Executes the full sequence from approach to insertion.

### `setup_peg_in_gripper.py`
Setup script to place peg in gripper and close it. Useful for starting with peg already grasped.

---

## 🎓 Learning Resources

- **UR5 Documentation**: [Universal Robots](https://www.universal-robots.com/)
- **ROS2 Humble**: [ROS2 Documentation](https://docs.ros.org/en/humble/)
- **MoveIt2**: [MoveIt Documentation](https://moveit.ros.org/)
- **Gazebo**: [Gazebo Documentation](http://gazebosim.org/)

---

## 📄 License

MIT License - See LICENSE file for details

---

## 👤 Author

AbdulqadeerTHD

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

## 📧 Contact

For issues and questions, please open an issue on GitHub.

---

## 🎉 Acknowledgments

- Universal Robots for UR5 robot model
- Robotiq for gripper model
- ROS2 and MoveIt communities
- Gazebo simulation framework

