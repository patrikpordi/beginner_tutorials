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
   ros2 launch cpp_pubsub talker_listern.yaml talker_f:=50.0

   ros2 service call /change_string cpp_pubsub/srv/ChangeString "new_string: 'Update'"
   OR
   use rqt to call the service


## Results

Can be found results/
   cppcheck.txt
   cpplint.txt
   rqt_console.png


