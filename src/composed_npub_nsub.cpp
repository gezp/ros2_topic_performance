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

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "ros2_topic_performance/utils.hpp"
#include "ros2_topic_performance/publisher_node.hpp"
#include "ros2_topic_performance/subscriber_node.hpp"

using namespace ros2_topic_performance;

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  
  // get some extra parameters
  bool use_intra_process_comms;
  bool use_dedicated_executors;
  int publisher_num,subscriber_num;
  auto tmp_node = std::make_shared<rclcpp::Node>("tmp_node");
  tmp_node->declare_parameter("use_intra_process_comms", false);
  tmp_node->declare_parameter("use_dedicated_executors", false);
  tmp_node->declare_parameter("publisher_num", 1);
  tmp_node->declare_parameter("subscriber_num", 1);
  tmp_node->get_parameter("use_intra_process_comms", use_intra_process_comms);
  tmp_node->get_parameter("use_dedicated_executors", use_dedicated_executors);
  tmp_node->get_parameter("publisher_num", publisher_num);
  tmp_node->get_parameter("subscriber_num", subscriber_num);
  if(publisher_num < 1){
    publisher_num = 1;
  }
  if(subscriber_num < 1){
    subscriber_num = 1;
  }
  tmp_node.reset();
  // create nodes
  std::vector<rclcpp::Node::SharedPtr> nodes;
  for(int i=0; i <publisher_num; i++){
    char name[20];
    sprintf(name,"publisher%d",i);
    auto options = get_node_options(name,use_intra_process_comms);
    auto node = std::make_shared<PublisherNode>(options);
    nodes.push_back(node);
  }
  for(int i=0; i <subscriber_num; i++){
    char name[20];
    sprintf(name,"subscriber%d",i);
    auto options = get_node_options(name,use_intra_process_comms);
    auto node = std::make_shared<SubscriberNode>(options);
    nodes.push_back(node);
  }
  // spin
  multi_spin(nodes,use_dedicated_executors);
  rclcpp::shutdown();
  return 0;
}
