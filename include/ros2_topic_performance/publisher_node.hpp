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
#include "geometry_msgs/msg/polygon_stamped.hpp"

namespace ros2_topic_performance
{

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(rclcpp::NodeOptions options);
private:
  void on_timer();
private:
  // parameters
  bool use_unique_ptr_{false};
  bool enable_output_address_{true};
  int point_num_{10};
  int rate_{1};
  // data
  size_t count_;
  geometry_msgs::msg::Polygon polygon_;
  //
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // ros2_topic_performance

#endif  // ROS2_TOPIC_PERFORMANCE__PUBLISHER_NODE_HPP_
