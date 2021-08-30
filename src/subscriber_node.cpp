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

#include "ros2_topic_performance/subscriber_node.hpp"

#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros2_topic_performance
{

SubscriberNode::SubscriberNode(const std::string & name, rclcpp::NodeOptions options)
: Node(name, options)
{
  subscription_ = create_subscription<std_msgs::msg::String>(
    "topic",
    10,
    [this](std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received message: '%10.10s', address: 0x%lx\n",
        msg->data.c_str(), reinterpret_cast<std::uintptr_t>(msg.get()));
    });
}

}