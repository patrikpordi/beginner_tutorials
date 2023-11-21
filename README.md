# Author: Patrik Dominik Pördi
# ROS Humble C++ PubSub Example

## Overview

This ROS 2 package, "cpp_pubsub," is a C++ example that demonstrates basic publisher, subscriber and service functionality in ROS 2 Humble. It showcases how to publish and subscribe to messages in a ROS 2 system, and how to modify the published message using a service call.

## Prerequisites

Before using this package, make sure you have the following dependencies:

- ROS 2 Humble (Installation guide: [ROS 2 Installation](https://docs.ros.org/en/humble/Installation.html))
- [Colcon](https://colcon.readthedocs.io/en/released/)
- C++17 compatible compiler

## Building and Installation

To build the package, follow these steps:

1. Clone the package into your ROS 2 workspace's `src/` directory:

   ```bash
   cd ~/ros2_ws/src
   git clone -b  ros_services_logging_launch  https://github.com/patrikpordi/beginner_tutorials.git
   colcon build --packages-select cpp_pubsub
   source ~/ros2_ws/install/setup.bash

   In another terminals run:
   source ~/ros2_ws/install/setup.bash
   ros2 run cpp_pubsub talker
   ros2 run cpp_pubsub listener
   OR
   using launch file:
   ros2 launch cpp_pubsub talker_listern.yaml => talker_f:= Frequancy 50.0 is a decent      option
      ros2 launch cpp_pubsub talker_listern.yaml => talker_f:= 50.0


   ros2 service call /change_string cpp_pubsub/srv/ChangeString "new_string: 'Update'"
   OR
   use rqt to call the service

   ----
To bag all the topics during launch use
```bash
ros2 launch cpp_pubsub talker_listener_bagger.py 
```
OR
```bash
# To disable recording
ros2 launch cpp_pubsub talker_listener_bagger.py talker_f:=10.0 record_enabled:='False'
```
OR
```bash
# To update the frequency of the talker and change the save location of the bag file
ros2 launch cpp_pubsub talker_listener_bagger.py talker_f:=10.0 bag_file:='./src/cpp_pubsub/results/new_recording'
```

This will run the talker node and bag all the topics it publishes for 15sec
> :warning: A new bag will not be created if a file with the same name already exists

To replay the bag file along with a listener node run the following command
```bash
# This will play the bag file recorded in the results path 
ros2 launch cpp_pubsub talker_listener_bagger.py replay_only:='True' bag_file:='./src/cpp_pubsub/results/new_recording'
```
Run the unit tests by running the command 
```bash
colcon test --packages-select cpp_pubsub
cat log/latest_test/cpp_pubsub/stdout_stderr.log
```

## CppLint & CppCheck
   ```bash
   # Use the below command for cpp lint by moving to root directory of your workspace 
   cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cpplint.txt

   # Use the below command for cpp check by moving to root directory of your workspace
   cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) --check-config  &> results/cppcheck.txt
```



## Results

Can be found results/
   cppcheck.txt
   cpplint.txt
   rqt_console.png
   frames_2023-11-19_17.22.55.gv
   frames_2023-11-19_17.22.55.pdf
   new_recording


