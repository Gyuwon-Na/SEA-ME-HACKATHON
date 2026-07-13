cmake_minimum_required(VERSION 3.8)
project(bisa_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs aruco)

add_executable(bisa_autonomous_node src/bisa_autonomous_node.cpp)
ament_target_dependencies(
  bisa_autonomous_node
  rclcpp ament_index_cpp sensor_msgs std_msgs control_msgs yaml_cpp_vendor
)
target_include_directories(bisa_autonomous_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(bisa_autonomous_node ${OpenCV_LIBRARIES} yaml-cpp)

install(TARGETS bisa_autonomous_node DESTINATION lib/${PROJECT_NAME})
ament_package()
