#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tf2/LinearMath/Quaternion.h"

#include "map_pkg/spawn_model.hpp"
#include "map_pkg/utilities.hpp"


class GatesPublisher : public rclcpp_lifecycle::LifecycleNode
{
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawner_;
  std::string share_dir;

  struct Data {
    std::string map_name;
    double dx;
    double dy;
    double x;
    double y;
  } data;

public:
  explicit GatesPublisher(bool intra_process_comms = false) 
  : rclcpp_lifecycle::LifecycleNode("send_gates",
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    )
  {
    // Get share directory with ament 
    this->share_dir = ament_index_cpp::get_package_share_directory("map_pkg");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&)
  {
    LifecycleNode::deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&)
  {
    // Map parameters
    this->declare_parameter("map", "hexagon");
    this->declare_parameter("dx", 10.0);
    this->declare_parameter("dy", 10.0);

    // Gate parameters
    this->declare_parameter("x", 0.0);
    this->declare_parameter("y", 0.0);
    this->data.map_name = this->get_parameter("map").as_string();
    if (this->data.map_name != "hexagon" && this->data.map_name != "rectangle"){
      RCLCPP_ERROR(this->get_logger(), "Map parameter must be either hexagon or rectangle.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
    
    this->publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/gate_position", qos);
    this->spawner_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>(
      "/world/" + this->data.map_name + "/create");

    // Get parameters
    this->data.dx = this->get_parameter("dx").as_double();
    this->data.dy = this->get_parameter("dy").as_double();
    this->data.x = this->get_parameter("x").as_double();
    this->data.y = this->get_parameter("y").as_double();

    RCLCPP_INFO(this->get_logger(), "Service client created for /world/%s/create", 
               this->data.map_name.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State&)
  {
    RCLCPP_INFO(this->get_logger(), "Activating node and spawning gates");

    bool spawn_success = false;
    
    // If the position was passed, then use it
    if (this->data.x != 0.0 && this->data.y != 0.0) {
      spawn_success = this->spawn_gates(this->data.x, this->data.y);
    }
    else {
      if (this->data.map_name == "hexagon"){
        spawn_success = this->rand_hexagon_gate();
      }
      else if (this->data.map_name == "rectangle"){
        spawn_success = this->rand_rectangle_gate();
      }
    }

    if (!spawn_success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to spawn gates");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  bool spawn_gates(double x, double y, double th = 0.0);
  bool rand_hexagon_gate();
  bool rand_rectangle_gate();
};

bool GatesPublisher::rand_hexagon_gate(){
  // Declare random generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(-this->data.dx, this->data.dx);
  std::uniform_int_distribution<> dis_left_right(0,1);
  obstacle obs {0.0, 0.0, 0.0, 1.0, 1.0, obstacle_type::BOX};

  obs.y = sqrt(3)/2.0*this->data.dx - 0.5-0.0001;
  obs.y = obs.y * (dis_left_right(gen) == 0 ? 1 : -1);
  double th = obs.y > 0 ? M_PI/2.0 : -M_PI/2.0;
  do {
    obs.x = dis_x(gen);
  } while(!is_inside_map(obs, "hexagon", this->data.dx, this->data.dy));
  
  return this->spawn_gates(obs.x, obs.y, th);
}

bool GatesPublisher::rand_rectangle_gate(){
  // Declare random generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(-this->data.dx, this->data.dx);
  std::uniform_real_distribution<> dis_y(-this->data.dy, this->data.dy);
  std::uniform_int_distribution<> dis_edge(0, 3);

  obstacle obs {0.0, 0.0, 0., 1.0, 1.0, obstacle_type::BOX};
  double th = 0.0;

  do {
    int edge = dis_edge(gen);
    switch (edge){
      //Top horizontal edge 
      case 0:
        obs.x = dis_x(gen);
        obs.y = this->data.dy/2.0 - 0.5-0.0001;
        break;
      //Bottom horizontal edge
      case 1:
        obs.x = dis_x(gen);
        obs.y = -(this->data.dy/2.0 - 0.5-0.0001);
        th = 3.0/2.0*M_PI;
        break;
      //Right vertical edge
      case 2:
        obs.x = this->data.dx/2.0 - 0.5-0.0001;
        obs.y = dis_y(gen);
        th = M_PI;
        break;
      //Left vertical edge
      case 3:
        obs.x = -(this->data.dx/2.0 - 0.5-0.0001);
        obs.y = dis_y(gen);
        th = 2.0*M_PI;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Random number generator failed, extracted %d, but should be in [0,4)", edge);
        return false;
    }
  } while(!is_inside_map(obs, "rectangle", this->data.dx, this->data.dy));
  
  return this->spawn_gates(obs.x, obs.y, th);
}

bool GatesPublisher::spawn_gates(double x, double y, double th){
  std_msgs::msg::Header hh;
  geometry_msgs::msg::Pose pose;
  std::vector<geometry_msgs::msg::Pose> pose_array_temp;
  geometry_msgs::msg::PoseArray msg;

  // Set headers of messages
  hh.stamp = this->get_clock()->now();
  hh.frame_id = "/map";
  msg.header = hh;

  // Set position of the gate
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.05;

  tf2::Quaternion q;
  q.setRPY(0, 0, th);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  pose_array_temp.push_back(pose);
  
  // Add gate to the message
  msg.poses = pose_array_temp;

  // Publish message
  publisher_->publish(msg);

  // Spawn gate in gazebo
  std::string xml = std::string((
    std::istreambuf_iterator<char>(std::ifstream(this->share_dir + "/models/gate/model.sdf").rdbuf())), 
    std::istreambuf_iterator<char>());
  
  if (!spawn_model(this->get_node_base_interface(), this->spawner_, xml, pose, "gate", true)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to spawn gate model");
    return false;
  }
  
  return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GatesPublisher>();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}