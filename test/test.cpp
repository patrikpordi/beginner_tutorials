// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file test.cpp
 * @author Patrik Dominik Pördi (ppordi@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief TaskPlanningFixture is a test fixture class that is used to setup and
 * 
 */
class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("basic_test")) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    // Setup things that should occur before every test instance should go here

    // Example: Start a ROS 2 node for testing
    bool retVal = StartROSExec("cpp_pubsub", "Custom_Node_Publisher", "talker");
    ASSERT_TRUE(retVal);

    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    // Tear things that should occur after every test instance should go here

    // Stop the running ROS 2 node, if any.
    bool retVal = StopROSExec();
    ASSERT_TRUE(retVal);

    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH TEARDOWN");
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  std::stringstream cmd_ss, cmdInfo_ss, killCmd_ss;

  static constexpr int kExecNameMaxLen = 15;  // Maximum length for execName

  bool StartROSExec(const char* pkg_name, const char* node_name,
                    const char* exec_name) {
    // Build command strings
    cmd_ss << "ros2 run " << pkg_name << " " << exec_name
           << " > /dev/null 2> /dev/null &";
    cmdInfo_ss << "ros2 node info "
               << "/" << node_name << " > /dev/null 2> /dev/null";
    char execName[kExecNameMaxLen + 1];  // +1 for null terminator
    snprintf(execName, sizeof(execName), "%s", exec_name);

    // First, kill the ROS 2 node, in case it's still running.
    StopROSExec();

    // Start a ROS 2 node and wait for it to get ready
    int retVal = system(cmd_ss.str().c_str());
    if (retVal != 0) return false;

    retVal = -1;
    while (retVal != 0) {
      retVal = system(cmdInfo_ss.str().c_str());
      sleep(1);
    }
    return true;
  }

  bool StopROSExec() {
    if (killCmd_ss.str().empty()) return true;

    int retVal = system(killCmd_ss.str().c_str());
    return retVal == 0;
  }
};

TEST_F(TaskPlanningFixture, TrueIsTrueTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic
   */
  using std_msgs::msg::String;
  using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;
  bool hasData = false;
  SUBSCRIBER subscription = node_->create_subscription<String>(
      "custom_topic", 10,
      // Lambda expression begins
      [&](const std_msgs::msg::String& msg) {
        RCLCPP_INFO(node_->get_logger(), "I heard: '%s'", msg.data.c_str());
        hasData = true;
      });

  /*
   * 3.) check to see if we get data winhin 3 sec
   */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while (elapsed_time < 3s) {
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  EXPECT_TRUE(hasData);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), "DONE SHUTTING DOWN ROS");
  return result;
}
