#include <chrono> // This code uses <chrono>, which is part of the C++11 standard.
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "cpp_pubsub/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// using namespace std::chrono_literals;
using std::chrono_literals::operator""ms;

/// @brief This class is used to publish a string message to a topic
/// The class inherits from the rclcpp::Node class
/// The class has a publisher member function which publishes a string message
/// to a topic
/// The class has a timer member function which calls the publisher member
/// function at a specified frequency set using parameter server
/// The class has a service member function which updates the string message
/// to be published
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("Custom_Node_Publisher"), msg_("Suppppp broooooo") {
    // Declare and acquire parameters from the parameter server
    this->declare_parameter("talker_f", 1.0);

    auto frequency = this->get_parameter("talker_f");

    if (frequency.get_type() == rclcpp::PARAMETER_NOT_SET) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Time period Parameter not set");
      throw std::runtime_error("Time period Parameter not set");
    }
    // Create a publisher
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("custom_topic", 10);
    // Create a service server
    service_ = this->create_service<cpp_pubsub::srv::ChangeString>(
        "change_string",
        std::bind(&MinimalPublisher::change_string_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    if (msg_ == "Suppppp broooooo") {
      RCLCPP_WARN_STREAM(this->get_logger(), "Using Default String");
    }
    // Create a timer
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1 / frequency.get_value<double>()),
        std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  /// @brief This function is used to publish a string message to a topic
  void timer_callback() {
    auto message = std_msgs::msg::String();
    unsigned int seed =
        std::chrono::system_clock::now().time_since_epoch().count();
    int random_number = rand_r(&seed) % 100;
    message.data = msg_ + std::to_string(random_number);
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
    publisher_->publish(message);
  }
  /// @brief This function is used to update the string message to be published
  /// @param request
  /// @param response
  void change_string_callback(
      const std::shared_ptr<cpp_pubsub::srv::ChangeString::Request> request,
      std::shared_ptr<cpp_pubsub::srv::ChangeString::Response> response) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Service Request Received");
    msg_ = request->new_string;
    response->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(), ("Changing the string to: " + msg_));
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<cpp_pubsub::srv::ChangeString>::SharedPtr service_;
  std::string msg_;
};
/// @brief  Main function for the publisher node
/// @param argc
/// @param argv
/// @return
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  // alert user about shutdown
  RCLCPP_FATAL_STREAM_ONCE(rclcpp::get_logger("rclpcpp"), "Subscriber closed");
  rclcpp::shutdown();
  return 0;
}
