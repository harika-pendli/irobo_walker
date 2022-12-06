# irobo_walker
Walker algorithm
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---
# Obstacle Avoidance in ROS2

### Overview

This is a ROS Package that defines a basic path by avoiding obstacles within a certain range of distance using Gazebo and Turtlebot3-'WafflePi' with depth camera plugin.

### Dependencies/ Assumptions
- OS : Ubuntu 20.04 
- ROS2 Distro : ROS2 Galactic
- Package build type : ```ament_cmake ```
- Package dependencies : ```rclcpp```, ```std_msgs``` 
- ROS2 Galactic Installation : [link](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

## How to Run the ROS Package
### Build Instructions

In spawn_turtlebot3.launch.py file , give complete path file to change urdf_path = '.. /models/model.sdf'
```
cd <your_ROS2_ws>/src
git clone https://github.com/harika-pendli/irobo_walker.git
cd ..   
rosdep install -i --from-path src --rosdistro galactic -y

export GAZEBO_MODEL_PATH=`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/
export TURTLEBOT3_MODEL=waffle_pi

colcon build --packages-select roomba_ros2
source . install/setup.bash

```

### Run Launch File to run simulation
To run the Launch file for obstacle avoidance node,run:
```

ros2 launch irobo_walker turtlebot3_world.launch.py
```

For enabling Ros2bag files except camera:
```

ros2 launch irobo_walker turtlebot3_world.launch.py record_all_topics:=True
```

Rosbag result files:[link]https://drive.google.com/drive/folders/1VdDUQhkmkG13TxuRW1wh4rM2HK1hInVZ?usp=share_link

To run them separately, you can use:
```
ros2 bag record -o rosbag_files <topic>
```
To check topic information that is in the rosbag:git
```
ros2 bag info rosbag_files
```
Lastly, to play back the data from rosbags:
```
ros2 bag play rosbag_files
``` 
## Results
The results after running the following commands are stored in the results folder.

### rqt Console
```
 ros2 run rqt_console rqt_console

```
### cppcheck
Run the following command from the root directory of your ROS package
```
cppcheck --enable=all --std=c++17 ./src/*.cpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
### cpplint
Run the following command from the root directory of your ROS package
```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp > ./results/cpplint.txt
```
### Google Styling format
Run the following command from the directory where the .cpp files are present(src in this case)
```
clang-format -style=Google -i your_file.cpp
```
