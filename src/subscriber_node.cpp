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

SubscriberNode::SubscriberNode(rclcpp::NodeOptions options)
: Node("subscriber", options)
{
  declare_parameter("enable_output_address", false);
  declare_parameter("enable_output_delay", false);
  get_parameter("enable_output_address", enable_output_address_);
  get_parameter("enable_output_delay", enable_output_delay_);
  subscription_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
    "topic",
    10,
    [this](geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
      //
      auto cur_time = this->now();
      if(enable_output_address_){
        RCLCPP_INFO(this->get_logger(), "Received message: '%8.5f', address: 0x%lx\n",
          msg->polygon.points[0].x, reinterpret_cast<std::uintptr_t>(msg.get()));
      }
      //printf latency
      if(enable_output_delay_){
        auto diff = cur_time - msg->header.stamp;
        auto delay = 0.001 * diff.nanoseconds();
        recv_num_ ++;
        avg_delay_us_ = avg_delay_us_ + (delay-avg_delay_us_)/recv_num_;
        min_delay_us_ = std::min(delay, min_delay_us_);
        max_delay_us_ = std::max(delay, max_delay_us_);
        RCLCPP_INFO(this->get_logger(), "delay info: cur[%.3f],avg[%.3f],min[%.3f],max[%.3f]x\n",
            delay,avg_delay_us_,min_delay_us_,max_delay_us_);
        //min_delay_us_;
      }
    });
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_topic_performance::SubscriberNode)