#include "map_pkg/spawn_model.hpp"

bool spawn_model(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_interface,
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr& spawner_,
  std::string xml, 
  geometry_msgs::msg::Pose pose, 
  std::string prefix, 
  bool wait)
{
  // Check if service is already available
  if (!spawner_->service_is_ready()) {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Waiting for spawn_entity service...");
    
    auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(10);
    
    while (!spawner_->wait_for_service(std::chrono::milliseconds(100))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service");
        return false;
      }
      
      // Only log every 2 seconds to avoid spamming
      auto current_time = std::chrono::steady_clock::now();
      if (current_time - start_time > timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out waiting for service");
        return false;
      }
      
      if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() % 2 == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Waiting for service...");
      }
    }
  }

  // Configure request
  auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
  request->entity_factory.name = prefix + std::to_string(pose.position.x) + "_" + std::to_string(pose.position.y);
  request->entity_factory.sdf = xml;
  request->entity_factory.pose = pose;
  request->entity_factory.relative_to = "world";

  // Send request
  auto result = spawner_->async_send_request(request);
  
  if (wait) {
    if (rclcpp::spin_until_future_complete(node_interface, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Spawned %s at x: %f, y: %f", 
                 prefix.c_str(), pose.position.x, pose.position.y);
      return true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service spawn_entity");
      return false;
    }
  }
  
  return true;
}