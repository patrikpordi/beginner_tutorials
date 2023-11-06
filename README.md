# Author: Patrik Dominik Pördi
# ROS Humble C++ PubSub Example

## Overview

This ROS 2 package, "cpp_pubsub," is a C++ example that demonstrates basic publisher and subscriber functionality in ROS 2 Humble. It showcases how to publish and subscribe to messages in a ROS 2 system.

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
   git clone https://github.com/patrikpordi/beginner_tutorials
   colcon build --packages-select cpp_pubsub
   source ~/ros2_ws/install/setup.bash

   In another terminals run:
   ros2 run cpp_pubsub talker
   ros2 run cpp_pubsub listener

## Results

Can be found in cppcheck.txt and cpplint.txt


