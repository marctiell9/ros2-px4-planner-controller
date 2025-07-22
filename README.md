# ros2-px4-planner-controller

## PX4 SITL Setup Tutorial

### 1. Clone the PX4-Autopilot Repository
```bash
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws/src/
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```
### 2. PX4-ROS2 integration
#### 1. Enable the ROS2-PX4 communication
```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

#### 2. Install the message packages for encoding and decoding PX4 messages
```bash
cd ~/px4_ros2_ws/src/
git clone https://github.com/PX4/px4_msgs.git
```

## ROS2 Simulation 
#### 1. Clone the repository
```bash
cd ~/px4_ros2_ws/src/
git clone https://github.com/marctiell9/ros2-px4-planner-controller.git
```
#### 2. Build the ROS 2 Workspace
```bash
cd ~/px4_ros2_ws/
colcon build
source install/setup.bash
```

#### 3. Launch the simulation
To run the full simulation, open three separate terminals and execute the following commands:

**Terminal 1 — Start PX4 SITL with Gazebo**
```bash
cd ~/px4_ros2_ws/src/PX4-Autopilot/
make px4_sitl gz_x500
```

**Terminal 2 — Start PX4-ROS2 Communication Bridge**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3 — Launch Planner and Controller**
```bash
cd ~/px4_ros2_ws/
source install/setup.bash
ros2 launch trajectory_planner_py planner.launch.py
```

## References
[1] T. Lee, M. Leok, N.H. McClamroch, “Geometric tracking control of quadrotor UAV on SE(3),” 49th IEEE Conference on Decision and Control, 2010

[2] D. Mellinger, V. Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors". In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), Shanghai, China, May 9–13, 2011
