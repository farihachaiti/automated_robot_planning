#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp> 

/**
 * @brief Spawn model of the gate in gazebo
 * 
 * @param pose The position in which the model should be spawned
 */
void spawn_model(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_interface,
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr& spawner_,
  std::string xml, geometry_msgs::msg::Pose pose, 
  std::string prefix = "obstacle", bool wait = false
);