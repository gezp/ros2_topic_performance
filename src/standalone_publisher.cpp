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

#include "ros2_topic_performance/publisher_node.hpp"
#include "ros2_topic_performance/subscriber_node.hpp"

using namespace ros2_topic_performance;

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto pub1 = std::make_shared<PublisherNode>(options);
  // spin
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub1);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
