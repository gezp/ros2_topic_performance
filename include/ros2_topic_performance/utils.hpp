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

#ifndef ROS2_TOPIC_PERFORMANCE__UTILS_HPP_
#define ROS2_TOPIC_PERFORMANCE__UTILS_HPP_

#include <thread>
#include "rclcpp/rclcpp.hpp"

namespace ros2_topic_performance
{

template<typename NodeT>
std::shared_ptr<std::thread> create_spin_thread(NodeT node){
  return std::make_shared<std::thread>([node](){
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node->get_node_base_interface());
      executor.spin();
      executor.remove_node(node->get_node_base_interface());
  });
}

}  // namespace ros2_topic_performance

#endif  // ROS2_TOPIC_PERFORMANCE__UTILS_HPP_
