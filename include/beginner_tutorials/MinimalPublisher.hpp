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
#include "beginner_tutorials/srv/rename_string.hpp"

using namespace std::chrono_literals; // for use of time units: "ms", "s"
//using std::placeholders::_1;          // for use with binding Class member
//using std::placeholders::_2;          // callback function


// topic types
//using STRING    = std_msgs::msg::String;
//using PUBLISHER = rclcpp::Publisher<STRING>::SharedPtr;
//using TIMER     = rclcpp::TimerBase::SharedPtr;

// service types
//using SERVICE    = rclcpp::Service<beginner_tutorials::srv::RenameString>::SharedPtr;
//using RENAME_STRING = beginner_tutorials::srv::RenameString;
//using REQUEST    = const std::shared_ptr<beginner_tutorials::srv::RenameString::Request>;
//using RESPONSE    = std::shared_ptr<beginner_tutorials::srv::RenameString::Response>;


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
  rclcpp::Service<beginner_tutorials::srv::RenameString>::SharedPtr service_;
  void change_base_string_srv( const std::shared_ptr<beginner_tutorials::srv::RenameString::Request> Request ,
      std::shared_ptr<beginner_tutorials::srv::RenameString::Response> Response) ;   // CHANGE
};
