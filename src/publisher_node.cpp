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

#include "ros2_topic_performance/publisher_node.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace ros2_topic_performance{


PublisherNode::PublisherNode(const std::string & name, rclcpp::NodeOptions options)
: Node(name, options), count_(0)
{
  declare_parameter("rate", 1);
  declare_parameter("message_length", 10);
  declare_parameter("use_unique_ptr", false);
  int message_length,rate;
  get_parameter("rate", rate);
  get_parameter("message_length", message_length);
  get_parameter("use_unique_ptr", use_unique_ptr_);
  data_ = std::string(message_length,'a');
  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / rate));
  timer_ = create_wall_timer(period_ms, std::bind(&PublisherNode::on_timer, this));
}

void PublisherNode::on_timer()
{
  if(use_unique_ptr_){
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = std::to_string(count_++) + "-" + data_;
    RCLCPP_INFO(this->get_logger(), "Published message: '%10.10s', address: 0x%lx \n",
      msg->data.c_str(), reinterpret_cast<std::uintptr_t>(msg.get()));
    publisher_->publish(std::move(msg));
  }else{
    std_msgs::msg::String msg;
    msg.data = std::to_string(count_++) + "-" + data_;
    RCLCPP_INFO(this->get_logger(), "Published message: '%10.10s', address: 0x%lx \n",
      msg.data.c_str(), reinterpret_cast<std::uintptr_t>(&msg));
    publisher_->publish(msg);
  }

}

}