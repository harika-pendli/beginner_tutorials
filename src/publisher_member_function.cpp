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

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

#include <signal.h>

#include <chrono>

#include "../include/beginner_tutorials/MinimalPublisher.hpp"

auto main_string = std::string("This is my main string");

/**
 * @brief Construct a new Minimal Publisher:: Minimal Publisher object
 *
 */
MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0) {
  publisher_ = this->create_publisher<STRING>("topic", 10);
  // Setting up parameter for publisher frequency
  auto freq_d = rcl_interfaces::msg::ParameterDescriptor();
  freq_d.description = "Sets Publisher frequency in Hz.";
  this->declare_parameter("freq_pub", 3.0, freq_d);
  auto freq_pub =
      this->get_parameter("freq_pub").get_parameter_value().get<std::float_t>();

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000 / freq_pub)),
      std::bind(&MinimalPublisher::timer_callback, this));

  auto serviceCallbackPtr =
      std::bind(&MinimalPublisher::change_base_string_srv, this,
                std::placeholders::_1, std::placeholders::_2);

  service_ = create_service<RENAME_STRING>("update_string", serviceCallbackPtr);

  // checks if any subscribers are already listening
  if (this->count_subscribers("topic") == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "No subscriber found on this topic");
  }
  this->get_logger().set_level(rclcpp::Logger::Level::Debug);

  tf_static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "talk";

  // Translation component in meters
  t.transform.translation.x = 0.1;
  t.transform.translation.y = 0.2;
  t.transform.translation.z = 0.3;

  // Quaternion corresponding to XYZ Euler Angles
  t.transform.rotation.x = 0.05;
  t.transform.rotation.y = 0.03;
  t.transform.rotation.z = 0.02;
  t.transform.rotation.w = 0.1;

  tf_static_broadcaster_->sendTransform(t);
}

void MinimalPublisher::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = main_string + std::to_string(count_++);
  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
  publisher_->publish(message);
}

/**
 * @brief Service that changes the base string to the requested message.
 *
 * @param request
 * @param response
 */
void MinimalPublisher::change_base_string_srv(REQUEST request,
                                              RESPONSE response) {
  response->out = request->inp;
  if (response->out == main_string) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Tried debug stream");
  }
  main_string = response->out;
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Incoming request" << request->inp);  // CHANGE
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "sending back response:" << response->out);
}

/**
 * @brief returns an error stream message when forced to shutdown using ctrl+c
 *
 * @param signum
 */
void node_forcestop(int signum) {
  if (signum == 2) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Force stopped! Bye!");
  }
}

int main(int argc, char* argv[]) {
  signal(SIGINT, node_forcestop);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
