/**
 * @file navigate.cpp
 * @author Harika (hpendli@umd.edu)
 * @brief File to avoid obstacles within range
 * @version 0.1
 * @date 2022-12-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using IMAGE = sensor_msgs::msg::Image;
using TWIST = geometry_msgs::msg::Twist;

typedef enum {
  FORWARD = 0,
  STOP,
  TURN,
} StateType;

/**
 * @brief irobo_walker Class
 *
 */
class irobo_walker : public rclcpp::Node {
 public:
  irobo_walker() : Node("walker"), state_(STOP) {
    // creates publisher to publish /cmd_vel topic
    auto pubTopicName = "cmd_vel";
    publisher_ = this->create_publisher<TWIST>(pubTopicName, 10);

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    // creates subscriber to get /demo_cam/camera/depth_demo topic
    auto subTopicName = "/demo_cam/camera/depth_demo";
    auto subCallback = std::bind(&irobo_walker::subscribe_callback, this, _1);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subTopicName, default_qos, subCallback);

    // create a 10Hz timer for processing

    auto processCallback = std::bind(&irobo_walker::process_callback, this);
    timer_ = this->create_wall_timer(100ms, processCallback);
  }

 private:
  void subscribe_callback(const IMAGE& msg) { lastImg_ = msg; }

  void process_callback() {
    // Do nothing until the first data read
    if (lastImg_.header.stamp.sec == 0) {
      return;
    }

    // Create the message to publish (initialized to all 0)
    auto message = TWIST();
    // state machine (Mealy -- output on transition)
    switch (state_) {
      case FORWARD:
        if (hasObstacle()) {  // check transition
          state_ = STOP;
          publisher_->publish(message);
          RCLCPP_INFO_STREAM(this->get_logger(), "State = STOP");
        }
        break;
      case STOP:
        if (hasObstacle()) {  // check transition
          state_ = TURN;
          message.angular.z = 0.1;
          publisher_->publish(message);
          RCLCPP_INFO_STREAM(this->get_logger(), "State = TURN");
        } else {
          state_ = FORWARD;
          message.linear.x = 0.1;
          publisher_->publish(message);
          RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
        }
        break;
      case TURN:
        if (!hasObstacle()) {  // check transition
          state_ = FORWARD;
          message.linear.x = 0.1;
          publisher_->publish(message);
          RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
        }
        break;
    }
  }

  /**
   * @brief hasObstacle() function return true if obstacle detected
   *
   *
   */

  bool hasObstacle() {
    unsigned char* dataPtr = lastImg_.data.data();
    float* floatData = (float*)dataPtr;

    int idx;
    for (unsigned int row = 0; row < lastImg_.height - 40; row++)
      for (unsigned int col = 0; col < lastImg_.width; col++) {
        idx = (row * lastImg_.width) + col;
        if (floatData[idx] < 1.0) {
          RCLCPP_INFO(this->get_logger(),
                      "row=%d, col=%d, floatData[idx] = %.2f", row, col,
                      floatData[idx]);
          return true;
        }
      }

    return false;
  }

  ////////////////////////////////////////
  // member variables
  ////////////////////////////////////////
  rclcpp::Subscription<IMAGE>::SharedPtr subscription_;
  rclcpp::Publisher<TWIST>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  IMAGE lastImg_;
  StateType state_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<irobo_walker>());
  rclcpp::shutdown();
  return 0;
}
