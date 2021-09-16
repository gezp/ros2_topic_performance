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

#ifndef ROS2_TOPIC_PERFORMANCE__SUBSCRIBER_NODE_HPP_
#define ROS2_TOPIC_PERFORMANCE__SUBSCRIBER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

namespace ros2_topic_performance
{

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode(rclcpp::NodeOptions options);

private:
  // parameters
  bool enable_output_address_{true};
  bool enable_output_delay_{true};
  // data
  int recv_num_{0};
  double min_delay_us_{100000};
  double max_delay_us_{0};
  double avg_delay_us_{0};
  //
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscription_;
};

}  // namespace ros2_topic_performance

#endif  // ROS2_TOPIC_PERFORMANCE__SUBSCRIBER_NODE_HPP_
