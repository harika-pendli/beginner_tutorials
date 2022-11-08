/**
 * @file MinimalPublisher.hpp
 * @author Harika Pendli (hpendli@umd.edu)
 * @brief Class implementation of the Minimal publisher
 * @version 0.1
 * @date 2022-11-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @brief Class (subclass of Node) and uses std::bind() to register a member
 * function as a callback from the timer.
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher();

 private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};
