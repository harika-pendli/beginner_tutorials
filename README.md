[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---

# ROS2 Publisher/Subscriber 

## Overview 

Creation of a ROS2 package and modification of the example publisher and sunscriber scripts to publish a custom string message, whilst following the Google C++ Style Guide. 

## Assumptions
* OS: Ubuntu Linux Focal (20.04) 64-bit
* ROS2 Distro: Humble
* ROS2 Workspace name: ros2_ws 
* ROS2 Installation Directory: ros2_humble

## ROS2 Dependencies
* ```ament_cmake```
* ```rclcpp```
* ```std_msgs```

## ROS2 Installation (source)

The following steps walkthrough the procedure to install the lastest LTS version of ROS2 (Humble) on an Ubuntu 20.04 machine, from source code. These steps can be found in [this link](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html).

If your system is running Ubuntu Linux Jammy (22.04) 64-bit, you may skip to the binary installation of ROS2 Humble using 
[this link.](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)


### ROS2 Code

Create an installation workspace and clone all repos. It is to be noted that it is assumed this installation workspace is created in the ```home``` directory of your system. If you have created it elsewhere, include the entire path to this directory. 

```
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

sudo apt upgrade
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

colcon build --symlink-install
```

NOTE: The previous step can take about 2+ hours to run.

### Environment Setup
```
. <path-to-ROS2-installation>/ros2_humble/install/local_setup.bash
```

### Clang
```
sudo apt install clang
export CC=clang
export CXX=clang++
colcon build --cmake-force-configure
```

### ROS2 Workspace
Here, an overlay workspace on top of the underlay installation workspace shall be created to place the custom-defined ROS2 packages. 
```
. <path-to-ROS2-installation>/ros2_humble/install/local_setup.bash
mkdir -p <path-to-ROS2-workspace>/ros2_ws/src
cd <path-to-ROS2-workspace>/ros2_ws/src
```
Source the 'underlay' installation workspace followed by the 'overlay',
```
. <path-to-ROS2-installation>/ros2_humble/install/local_setup.bash
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
```

## Build Instructions
```
cd <path-to-ROS2-workspace>/ros2_ws/src
git clone https://github.com/harika-pendli/beginner_tutorials.git
cd ..  
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select beginner_tutorials
```

## Run Instructions

### Publisher Demo 
In a new terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 run beginner_tutorials talker
```

### Subscriber Demo
In another terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 run beginner_tutorials listener
```

Enter ```Ctrl+c``` in each terminal to stop the nodes from spinning.


## Results

### cpplint 
Change to the root directory of the package, ```/beginner_tutorials```, and run:
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp ./include/beginner_tutorials/*.hpp > ./results/cpplint.txt
```
The results of running ```cpplint``` can be found in ```/results/cpplint.txt```.

### cppcheck
Change to the root directory of the package, ```/beginner_tutorials```, and run:
```
cppcheck --enable=all --std=c++17 ./src/*.cpp ./include/beginner_tutorials/*.hpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
The results of running ```cppcheck``` can be found in ```/results/cppcheck.txt```.

### Google style fomat

To format code to Google C++ Sytle, run this command for each .cpp/.hpp file. 
```
  clang-format -style=Google -i your_file.cpp
```