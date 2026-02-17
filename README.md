# ViSP + ROS 2 Jazzy + CoppeliaSim Workspace

**OS:** Ubuntu 24.04 LTS (Noble Numbat)
**ROS Version:** ROS 2 Jazzy
**Simulator:** CoppeliaSim Edu V4.10
**Status:** Patched & Compatible

This repository contains a ready-to-use workspace for **ViSP** (Visual Servoing Platform) integrated with **ROS 2 Jazzy** and **CoppeliaSim**. It includes the `visp_ros` package, pre-patched to fix header compatibility issues in ROS 2 Jazzy (e.g., `.h` vs `.hpp`) and configured with a custom CMake setup for proper dependency resolution.

## 1. Prerequisites

Ensure you have the following installed on your system:
* **Ubuntu 24.04 LTS**
* **ROS 2 Jazzy Jalisco** (Desktop Install)
* **CoppeliaSim Edu V4.10** (Download from Coppelia Robotics)

### 1.1 Install System Dependencies
Install the required ROS 2 packages and vision libraries:

```bash
sudo apt update
sudo apt install ros-jazzy-visp ros-jazzy-vision-visp ros-jazzy-image-geometry ros-jazzy-cv-bridge ros-jazzy-pcl-ros libopencv-dev python3-colcon-common-extensions 
```

2. Installation
2.1 Clone the Repository
Clone this workspace to your home directory:

```Bash
cd ~
git clone [https://github.com/aymenbll/VISP_Coppeliasim_ws.git](https://github.com/aymenbll/VISP_Coppeliasim_ws.git) coppelia_ros2_ws
```
2.2 Build the Workspace
Since the code patches and CMakeLists.txt fixes are already included in this repo, you just need to build it.

```Bash
cd ~/coppelia_ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
3. Critical Step: Recompile CoppeliaSim ROS 2 Interface
The default ROS 2 plugin shipped with CoppeliaSim V4.10 on Jazzy does not support the camera and joint messages required for this simulation (it causes "Unsupported message type" errors). You must recompile it manually to enable these features.

3.1 Prepare a Temporary Plugin Workspace
Create a workspace to build the plugin:

```Bash
mkdir -p ~/plugin_ws/src
cd ~/plugin_ws/src
# Link the source code from your CoppeliaSim installation
# REPLACE '$HOME/CoppeliaSim' with the actual path to your simulator folder
ln -s $HOME/CoppeliaSim/programming/ros2_packages/sim_ros2_interface .
```

3.2 Add Missing Interfaces
Open the interface definition file:

```Bash
nano ~/plugin_ws/src/sim_ros2_interface/meta/interfaces.txt
```
Append the following lines to the end of the file (do not delete existing ones):

```Plaintext
sensor_msgs/msg/Image
sensor_msgs/msg/CameraInfo
sensor_msgs/msg/JointState
geometry_msgs/msg/Pose
geometry_msgs/msg/WrenchStamped
geometry_msgs/msg/Inertia
geometry_msgs/msg/Vector3
std_msgs/msg/Int32
```

3.3 Compile the Plugin
Build the plugin, explicitly pointing CMake to your CoppeliaSim installation path.

```Bash
cd ~/plugin_ws
export COPPELIASIM_ROOT_DIR=$HOME/CoppeliaSim
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCOPPELIASIM_ROOT_DIR=$COPPELIASIM_ROOT_DIR \
  -DCoppeliaSim_DIR=$COPPELIASIM_ROOT_DIR/programming/libUtils/cmake
```

3.4 Install the New Plugin
Copy the generated library to your CoppeliaSim root folder (replacing the old one).

```Bash
cp ~/plugin_ws/build/sim_ros2_interface/libsimROS2.so $HOME/CoppeliaSim/
```

4. Running the Simulation
4.1 Start CoppeliaSim
Open a terminal and start the simulator:

```Bash
cd ~/CoppeliaSim
./coppeliaSim.sh
```

Verify Plugin Load: Check the terminal output for the line:
Plugin 'ROS2': load succeeded.

Load Scene: Inside CoppeliaSim, go to File > Open Scene... and navigate to:
~/coppelia_ros2_ws/src/visp_ros/scene/franka-panda.ttt


4.2 Run the Visual Servoing Node
Open a new terminal to run the control node. This node will process the camera image and send velocity commands to the robot.

```Bash
cd ~/coppelia_ros2_ws
source install/setup.bash
```

# Run the Position-Based Visual Servoing (PBVS) tutorial
```Bash
ros2 run visp_ros tutorial-franka-coppeliasim-pbvs-apriltag --plot --enable-coppeliasim-sync-mode
```
