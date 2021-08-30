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

#ifndef ROS2_TOPIC_PERFORMANCE__PUBLISHER_NODE_HPP_
#define ROS2_TOPIC_PERFORMANCE__PUBLISHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros2_topic_performance
{

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(const std::string & name, rclcpp::NodeOptions options);
private:
  void on_timer();
  size_t count_;
  bool use_unique_ptr_{false};
  std::string data_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // ros2_topic_performance

#endif  // ROS2_TOPIC_PERFORMANCE__PUBLISHER_NODE_HPP_
