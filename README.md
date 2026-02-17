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

# Run the Position-Based Visual Servoing (PBVS) tutorial

ros2 run visp_ros tutorial-franka-coppeliasim-pbvs-apriltag --plot --enable-coppeliasim-sync-mode
```


# Wanna do it from scratch

Visual Servoing (ViSP) + ROS 2 Jazzy + CoppeliaSim Tutorial
OS: Ubuntu 24.04 LTS (Noble Numbat)
ROS Version: ROS 2 Jazzy Jalisco
Simulator: CoppeliaSim Edu V4.10
Status: Patched & Compatible

This guide provides a complete, step-by-step tutorial on building the entire ViSP ROS stack (vision_visp and visp_ros) from source on ROS 2 Jazzy. It covers installing dependencies, patching the source code for Jazzy compatibility (fixing .h vs .hpp includes), configuring the build system, and recompiling the CoppeliaSim ROS 2 interface.

1. Prerequisites
Before starting, ensure you have:

Ubuntu 24.04 LTS installed.

ROS 2 Jazzy installed (Desktop version).

CoppeliaSim Edu V4.10 downloaded and extracted to your home folder.

1.1 Install Dependencies
We need the core ViSP library and build tools.

```Bash
sudo apt update
# Install ViSP core and build tools
sudo apt install ros-jazzy-visp ros-jazzy-pcl-ros libopencv-dev python3-colcon-common-extensions git
```
2. Setting Up the Workspace
We will create a workspace and clone both vision_visp (provides the bridge to ViSP types) and visp_ros (provides the robot controllers).

2.1 Create the Directory
```Bash
mkdir -p ~/visp_ws/src
cd ~/visp_ws/src
```
2.2 Clone Source Repositories
We use the rolling branch for both repositories as it is the closest to supporting modern ROS 2 versions.

```Bash
# Clone vision_visp (visp_bridge, visp_camera_calibration, etc.)
git clone -b rolling https://github.com/lagadic/vision_visp.git

# Clone visp_ros (nodes and robot interfaces)
git clone -b rolling https://github.com/lagadic/visp_ros.git
```
3. Patching Source Code for ROS 2 Jazzy
ROS 2 Jazzy introduced breaking changes by renaming header files from .h to .hpp in cv_bridge and image_geometry. We must patch both packages to fix these includes.

Run these commands from your workspace root (~/visp_ws):

```Bash
cd ~/visp_ws

# 1. Apply global fix for cv_bridge headers (Affects both vision_visp and visp_ros)
grep -rl "cv_bridge/cv_bridge.h" src/ | xargs sed -i 's|cv_bridge/cv_bridge.h|cv_bridge/cv_bridge.hpp|g'

# 2. Apply global fix for image_geometry headers
grep -rl "image_geometry/pinhole_camera_model.h" src/ | xargs sed -i 's|image_geometry/pinhole_camera_model.h|image_geometry/pinhole_camera_model.hpp|g'

# 3. Specific fix for Blob Tracker in visp_ros (sometimes missed by generic grep)
sed -i 's|cv_bridge/cv_bridge.h|cv_bridge/cv_bridge.hpp|g' src/visp_ros/nodes/blob_tracker.cpp
```

4. Configuring the Build System
The default CMakeLists.txt in visp_ros often fails to find dependencies automatically. We will replace it with a robust configuration.

Open the file:

```Bash
nano ~/visp_ws/src/visp_ros/CMakeLists.txt
```
Delete everything in the file and paste the following content:

```CMake
cmake_minimum_required(VERSION 3.5)
project(visp_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visp_bridge REQUIRED)
find_package(VISP REQUIRED COMPONENTS visp_core visp_robot)
find_package(orocos_kdl QUIET)
find_package(iir QUIET)

include_directories(
  "/opt/ros/jazzy/include"
  "/usr/include/opencv4"
  include
  ${VISP_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BlobTracker.msg"
  "msg/ImagePoint.msg"
  "msg/PoseStampedStatus.msg"
  "msg/ProjectedPoint.msg"
  DEPENDENCIES sensor_msgs geometry_msgs
)

set(orocos_kdl_VERSION_MINIMUM "1.3.2")
if(orocos_kdl_FOUND AND(${orocos_kdl_VERSION} VERSION_GREATER ${orocos_kdl_VERSION_MINIMUM}))
  add_definitions("-DVISP_HAVE_OROCOS_KDL")
  if(iir_FOUND)
    add_definitions("-DVISP_HAVE_IIR")
  endif()
endif()

set(srcs src/device/framegrabber/vpROSGrabber.cpp src/robot/vpROSRobot.cpp)

if(orocos_kdl_FOUND AND(${orocos_kdl_VERSION} VERSION_GREATER ${orocos_kdl_VERSION_MINIMUM}))
  list(APPEND srcs
    src/robot/sim-robot/franka/vpRobotFrankaSim.cpp
    src/robot/sim-robot/franka/vpROSRobotFrankaCoppeliasim.cpp
    src/robot/sim-robot/franka/model/CoriolisMatrix.cpp
    src/robot/sim-robot/franka/model/franka_model.h
    src/robot/sim-robot/franka/model/FrictionTorque.cpp
    src/robot/sim-robot/franka/model/GravityVector.cpp
    src/robot/sim-robot/franka/model/MassMatrix.cpp
  )
endif()

add_library(${PROJECT_NAME}_common ${srcs})
ament_target_dependencies(${PROJECT_NAME}_common cv_bridge geometry_msgs image_geometry image_transport joy nav_msgs rclcpp sensor_msgs std_srvs visp_bridge VISP orocos_kdl)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_common "${cpp_typesupport_target}")
target_include_directories(${PROJECT_NAME}_common PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>" ${VISP_INCLUDE_DIRS})

# Nodes
add_executable(${PROJECT_NAME}_afma6_node nodes/afma6.cpp)
target_link_libraries(${PROJECT_NAME}_afma6_node "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_afma6_node geometry_msgs rclcpp visp_bridge VISP)

add_executable(${PROJECT_NAME}_joy2twist_afma6_node nodes/joy2twist_afma6.cpp)
target_link_libraries(${PROJECT_NAME}_joy2twist_afma6_node "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_joy2twist_afma6_node geometry_msgs sensor_msgs rclcpp joy VISP)

add_executable(${PROJECT_NAME}_blob_tracker_node nodes/blob_tracker.cpp)
target_link_libraries(${PROJECT_NAME}_blob_tracker_node "${cpp_typesupport_target}" ${PROJECT_NAME}_common)
ament_target_dependencies(${PROJECT_NAME}_blob_tracker_node cv_bridge rclcpp std_msgs visp_bridge VISP)

add_executable(${PROJECT_NAME}_viper650_node nodes/viper650.cpp)
target_link_libraries(${PROJECT_NAME}_viper650_node "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_viper650_node geometry_msgs rclcpp sensor_msgs visp_bridge VISP)

add_executable(${PROJECT_NAME}_viper850_node nodes/viper850.cpp)
target_link_libraries(${PROJECT_NAME}_viper850_node "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_viper850_node geometry_msgs rclcpp sensor_msgs visp_bridge VISP)

add_executable(tutorial-ros-grabber tutorial/grabber/ros/tutorial-ros-grabber.cpp)
target_link_libraries(tutorial-ros-grabber ${PROJECT_NAME}_common "${cpp_typesupport_target}")
ament_target_dependencies(tutorial-ros-grabber rclcpp sensor_msgs visp_bridge VISP)

if(orocos_kdl_FOUND AND(${orocos_kdl_VERSION} VERSION_GREATER ${orocos_kdl_VERSION_MINIMUM}))
  set(tutorial_franka
    tutorial/franka/coppeliasim/test-franka-coppeliasim-controller.cpp
    tutorial/franka/coppeliasim/tutorial-franka-coppeliasim-ibvs-apriltag.cpp
    tutorial/franka/coppeliasim/tutorial-franka-coppeliasim-pbvs-apriltag.cpp
    tutorial/franka/coppeliasim/tutorial-franka-coppeliasim-cartesian-impedance-control.cpp
    tutorial/franka/coppeliasim/tutorial-franka-coppeliasim-joint-impedance-control.cpp
    tutorial/franka/coppeliasim/tutorial-franka-coppeliasim-dual-arm.cpp
  )
  foreach(cpp ${tutorial_franka})
    get_filename_component(bin_name ${cpp} NAME_WE)
    add_executable(${bin_name} ${cpp})
    target_link_libraries(${bin_name} ${PROJECT_NAME}_common "${cpp_typesupport_target}")
    ament_target_dependencies(${bin_name} geometry_msgs rclcpp sensor_msgs visp_bridge VISP)
    install(TARGETS ${bin_name} DESTINATION lib/${PROJECT_NAME})
  endforeach()
endif()

install(TARGETS ${PROJECT_NAME}_common ${PROJECT_NAME}_afma6_node ${PROJECT_NAME}_blob_tracker_node ${PROJECT_NAME}_viper650_node ${PROJECT_NAME}_viper850_node ${PROJECT_NAME}_joy2twist_afma6_node tutorial-ros-grabber ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime VISP cv_bridge image_geometry)
ament_package()
```
Save and exit (Ctrl+O, Enter, Ctrl+X).

4.1 Build the Workspace
Build both packages (vision_visp and visp_ros):

```Bash
cd ~/visp_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

5. Recompiling the CoppeliaSim ROS 2 Plugin (Critical)
The default plugin in CoppeliaSim V4.10 on Jazzy lacks support for CameraInfo, Image, and JointState messages. We must recompile it to enable the simulation to talk to ROS.

5.1 Create Plugin Workspace
```Bash
mkdir -p ~/plugin_ws/src
cd ~/plugin_ws/src
# Link the source from your CoppeliaSim folder (Adjust path if needed)
ln -s $HOME/CoppeliaSim/programming/ros2_packages/sim_ros2_interface .
```
5.2 Add Missing Interfaces
Open interfaces.txt to add the required message types:

```Bash
nano ~/plugin_ws/src/sim_ros2_interface/meta/interfaces.txt
```
Append the following lines to the end of the file:

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
Save and exit.

5.3 Compile the Plugin
Point COPPELIASIM_ROOT_DIR to your installation folder and build.

```Bash
cd ~/plugin_ws
export COPPELIASIM_ROOT_DIR=$HOME/CoppeliaSim
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCOPPELIASIM_ROOT_DIR=$COPPELIASIM_ROOT_DIR \
  -DCoppeliaSim_DIR=$COPPELIASIM_ROOT_DIR/programming/libUtils/cmake
```

5.4 Install the Plugin
Copy the generated library (libsimROS2.so) to your CoppeliaSim root folder.

```Bash
cp ~/plugin_ws/build/sim_ros2_interface/libsimROS2.so $HOME/CoppeliaSim/
```
6. Running the Simulation
6.1 Start CoppeliaSim
Open a terminal and run:

```Bash
cd ~/CoppeliaSim
./coppeliaSim.sh
```
Check the terminal output. You should see: Plugin 'ROS2': load succeeded.

Inside CoppeliaSim, go to File > Open Scene...

Navigate to: ~/visp_ws/src/visp_ros/scene/franka-panda.ttt


6.2 Run the Visual Servoing Node
Open a new terminal to run the PBVS tutorial node:

```Bash
cd ~/visp_ws
source install/setup.bash

# Run the controller
ros2 run visp_ros tutorial-franka-coppeliasim-pbvs-apriltag --plot --enable-coppeliasim-sync-mode
```

##For Any Doubts Check gemini .biboy conversation "Fix CoppeliaSim ROS2 Build Error"
