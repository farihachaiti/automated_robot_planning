cmake_minimum_required(VERSION 3.8)
project(automated_robot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(shelfino_description REQUIRED)
find_package(shelfino_gazebo REQUIRED)
find_package(shelfino_navigation REQUIRED)
find_package(map_pkg REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shelfino_node REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
${PROJECT_NAME}/robot_controller.py
${PROJECT_NAME}/QosBridge.py
#${PROJECT_NAME}/dubins_node.py
  DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})



# Export dependencies
ament_export_dependencies(rclpy rclcpp obstacles_msgs shelfino_description shelfino_gazebo shelfino_navigation map_pkg shelfino_node)


ament_package()
