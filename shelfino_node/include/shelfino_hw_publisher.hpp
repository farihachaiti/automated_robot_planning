#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


#include "std_srvs/srv/set_bool.hpp"

#include "hardwareparameters.h"
#include "hardwareglobalinterface.h"
#include "json.hpp"

using namespace std::chrono_literals;
using namespace nlohmann;

/**
 * @brief Shelfino ROS2 node to bridge the sensor data from ZMQ to ROS2.
 * 
 * ROS 2 interface for the mobile robot Shelfino of the Department of Information Engineering and Computer Science of the University of Trento.
 */
class ShelfinoHWNode : public rclcpp::Node
{
  public:
    /**
     * @brief Construct a new Shelfino HW Node object.
     * 
     * Create all the ROS2 tranform broadcasters, topic publishers and subscribers.
     */
    ShelfinoHWNode();

  private:
    //rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr relay_scan_publisher_;
    //rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr relay_scan_subscription_;

    /**
     * @brief Method callback that retrieves the lidar scan data from ZMQ and publishes to ROS
     * 
     */
    void lidar_timer_callback();
    void lidar_callback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    // /**
    //  * @brief Method callback that retrieves the RealSense odometry data from ZMQ and publishes to ROS
    //  * 
    //  */
    // void t265_callback();

    void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    /**
     * @brief Method callback that retrieves odometry data from ZMQ and publishes to ROS
     * 
     * The odometry is calculated from the sensor fusion of encoders and RealSense data
     */
    void odom_timer_callback();

    // void odom_timer_callback();

    /**
     * @brief Method callback that retrieves the encoders data from ZMQ and publishes to ROS
     * 
     */
    void enc_callback();

    /**
     * @brief Method that handles the velocities commands from ROS to ZMQ.
     * 
     * @param msg The velocity mesasge used as control action for the robot.
     */
    void handle_shelfino_cmd_vel(const std::shared_ptr<geometry_msgs::msg::Twist> msg);

    /**
     * @brief Method that broadcasts the transform from the "odom" frame to "base_footprint" frame.
     * 
     * @param msg The odometry message containing the information about the robot's location.
     */
    void handle_shelfino_pose(nav_msgs::msg::Odometry msg);
    void t265_callback();

    void handle_power_srv(std_srvs::srv::SetBool::Request::SharedPtr request,
                          std_srvs::srv::SetBool::Response::SharedPtr response);
    
    // Core node members
    std::string ns;
    int shelfino_id;
    
    // ROS2 communication
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr t265_odom_subscription_;

    // ROS2 publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;  // Standard /odom topic
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoders_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr t265_publisher_;
    rclcpp::TimerBase::SharedPtr t265_timer_;
    // ROS2 subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    // Relay subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr relay_lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr relay_odom_sub_;
    // Relay publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr relay_lidar_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr relay_odom_pub_;
    // ROS2 timers
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    // Store latest received LaserScan for timer-based publishing
    sensor_msgs::msg::LaserScan latest_lidar_msg_;

    // TF2 related
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr encoders_timer_;

    // rclcpp::TimerBase::SharedPtr t265_timer_;
    
    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};