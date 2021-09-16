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

rclcpp::NodeOptions get_node_options(const std::string &name,bool use_intra_process_comms){
  rclcpp::NodeOptions options;
  if(!name.empty()){
    options.arguments({"--ros-args", "-r", std::string("__node:=") + name, "--"});
  }
  options.use_intra_process_comms(use_intra_process_comms);
  return options;
}

template<typename NodeT>
std::shared_ptr<std::thread> create_spin_thread(NodeT node){
  return std::make_shared<std::thread>([node](){
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node->get_node_base_interface());
      executor.spin();
      executor.remove_node(node->get_node_base_interface());
  });
}

void multi_spin(std::vector<rclcpp::Node::SharedPtr> nodes, bool use_dedicated_executors){
  if(use_dedicated_executors){
    std::vector<std::shared_ptr<std::thread>> threads;
    for(auto node: nodes){
      threads.push_back(create_spin_thread(node));
    }
    for(auto t: threads){
      t->join();
    }
  }else{
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),nodes.size());
    for(auto node: nodes){
      executor.add_node(node);
    }
    executor.spin();
  }
}

}  // namespace ros2_topic_performance

#endif  // ROS2_TOPIC_PERFORMANCE__UTILS_HPP_
