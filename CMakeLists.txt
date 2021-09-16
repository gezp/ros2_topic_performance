cmake_minimum_required(VERSION 3.5)

project(ros2_topic_performance)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(topic_nodes SHARED
            src/publisher_node.cpp
            src/subscriber_node.cpp)

ament_target_dependencies(topic_nodes rclcpp rclcpp_components geometry_msgs)

# Create  node
add_executable(standalone_publisher src/standalone_publisher.cpp)
target_link_libraries(standalone_publisher topic_nodes)
add_executable(standalone_subscriber src/standalone_subscriber.cpp)
target_link_libraries(standalone_subscriber topic_nodes)
#
add_executable(composed_npub_nsub src/composed_npub_nsub.cpp)
target_link_libraries(composed_npub_nsub topic_nodes)


rclcpp_components_register_nodes(topic_nodes "ros2_topic_performance::PublisherNode")
rclcpp_components_register_nodes(topic_nodes "ros2_topic_performance::SubscriberNode")

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS topic_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable nodes
install(TARGETS 
    standalone_publisher
    standalone_subscriber
    composed_npub_nsub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
