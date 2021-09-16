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


PublisherNode::PublisherNode(rclcpp::NodeOptions options)
: Node("publisher", options), count_(0)
{
  declare_parameter("rate", 1);
  declare_parameter("point_num", 10);
  declare_parameter("use_unique_ptr", false);
  declare_parameter("enable_output_address", false);
  get_parameter("rate", rate_);
  get_parameter("point_num", point_num_);
  get_parameter("use_unique_ptr", use_unique_ptr_);
  get_parameter("enable_output_address", enable_output_address_);
  // create msg data
  // data_ = std::string(point_num_,'a');
  geometry_msgs::msg::Point32 point;
  for(int i=0; i< point_num_; i++){
    point.x = point.y = point.z = 0.001*i;
    polygon_.points.push_back(point);
  }
  // create publisher and timer
  publisher_ = create_publisher<geometry_msgs::msg::PolygonStamped>("topic", 10);
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / rate_));
  timer_ = create_wall_timer(period_ms, std::bind(&PublisherNode::on_timer, this));
}

void PublisherNode::on_timer()
{
  // update data
  count_++;
  polygon_.points[0].x = 0.0001 * count_;
  auto cur_time = this->now();
  // publish
  if(use_unique_ptr_){
    auto msg = std::make_unique<geometry_msgs::msg::PolygonStamped>();
    msg->header.stamp = cur_time;
    msg->polygon = polygon_;
    if(enable_output_address_){
      RCLCPP_INFO(this->get_logger(), "Publish message: '%8.5f', address: 0x%lx\n",
        msg->polygon.points[0].x, reinterpret_cast<std::uintptr_t>(msg.get()));
    }
    publisher_->publish(std::move(msg));
  }else{
    geometry_msgs::msg::PolygonStamped msg;
    msg.header.stamp = cur_time;
    msg.polygon = polygon_;
    if(enable_output_address_){
      RCLCPP_INFO(this->get_logger(), "Publish message: '%8.5f', address: 0x%lx\n",
        msg.polygon.points[0].x, reinterpret_cast<std::uintptr_t>(&msg));
    }
    publisher_->publish(msg);
  }

}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_topic_performance::PublisherNode)