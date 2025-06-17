#include "shelfino_hw_publisher.hpp"

static std::unique_ptr<HardwareParameters> hp;

ShelfinoHWNode::ShelfinoHWNode(): Node("shelfino_hw_publisher")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  // Load shelfino paths to communicate with ZMQ
  this->declare_parameter("shelfino_id", 1);
  shelfino_id = this->get_parameter("shelfino_id").get_parameter_value().get<int>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "shelfino_id: %d", shelfino_id);

  hp = std::make_unique<HardwareParameters>(shelfino_id);
  HardwareGlobalInterface::initialize(hp.get());

  // ROS2 transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create QoS profile for odometry data with BEST_EFFORT reliability
  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  odom_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  odom_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // Create optimized QoS profile for LIDAR data
  auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(10));  // Balanced queue size
  lidar_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  lidar_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  lidar_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  
  // Set deadline to 100ms to match typical LIDAR update rates (10Hz)
  lidar_qos.deadline(std::chrono::milliseconds(50));
  
  // Configure liveliness to detect unresponsive publishers
  lidar_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
  lidar_qos.liveliness_lease_duration(std::chrono::seconds(1));
  
  // Optimize for throughput over latency
  lidar_qos.avoid_ros_namespace_conventions(false);

  // Creation of ROS2 publishers
  lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_best_effort", lidar_qos);
  t265_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("t265", qos);
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_best_effort", odom_qos);
  encoders_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

  // Selecting the callbacks for the publishers
  // Reduce Lidar publishing rate to 5Hz (200ms) to match typical LIDAR update rates
  lidar_timer_ = this->create_wall_timer(200ms, std::bind(&ShelfinoHWNode::lidar_timer_callback, this));
  t265_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWNode::t265_callback, this));
  odom_timer_ = this->create_wall_timer(10ms, std::bind(&ShelfinoHWNode::odom_timer_callback, this));
  encoders_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWNode::enc_callback, this));
  
  // Create a QoS profile that matches the ros_gz_bridge publisher
  auto odom_subscription_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  odom_subscription_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  odom_subscription_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


  auto lidar_subscription_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  lidar_subscription_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  lidar_subscription_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  // Subscribe to gazebo odometry with matching QoS
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", odom_subscription_qos, std::bind(&ShelfinoHWNode::odom_callback, this, std::placeholders::_1));

  // rs_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "pose/sample", 10, std::bind(&ShelfinoHWNode::handle_rs_odom, this, std::placeholders::_1));

  // Creation of the CMD_VEL subscriber to move the shelfino
  cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&ShelfinoHWNode::handle_shelfino_cmd_vel, this, std::placeholders::_1));

  //t265_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //  "pose/sample", rclcpp::SensorDataQoS(), std::bind(&ShelfinoHWNode::odom_callback, this, std::placeholders::_1));
    
  // Retrieve node namespace to use as prefix of transforms
  ns = this->get_namespace();
  ns.erase(0,1);

  service_ = this->create_service<std_srvs::srv::SetBool>("power", std::bind(&ShelfinoHWNode::handle_power_srv, this, std::placeholders::_1, std::placeholders::_2));

  // Add LaserScan relay subscriber and publisher
  //relay_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
  //  "scan", rclcpp::QoS(rclcpp::SensorDataQoS().best_effort()));

  lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", lidar_subscription_qos, std::bind(&ShelfinoHWNode::lidar_callback, this, std::placeholders::_1));

  // --- Relay publisher and subscriber for LIDAR ---
  /*relay_lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_best_effort", rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
  relay_lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
      relay_lidar_pub_->publish(*msg);
    }
  );

  // --- Relay publisher and subscriber for ODOM ---
  relay_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "odom_best_effort", rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
  relay_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      relay_odom_pub_->publish(*msg);
    }
  );*/
}


void ShelfinoHWNode::lidar_timer_callback()
{
  try {

      //latest_lidar_msg_.header.stamp = this->get_clock()->now();
      lidar_publisher_->publish(latest_lidar_msg_);
      RCLCPP_DEBUG(this->get_logger(), "Republished latest subscribed lidar scan with %zu ranges to frame: %s", latest_lidar_msg_.ranges.size(), latest_lidar_msg_.header.frame_id.c_str());

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error in lidar_callback (timer): %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown error in lidar_callback (timer)");
  }
}

void ShelfinoHWNode::lidar_callback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null Lidar message");
    return;
  }

  // Store the latest received LaserScan message
  latest_lidar_msg_ = *msg;
}




void ShelfinoHWNode::t265_callback()
{
  RobotStatus::OdometryData odomData; 
  HardwareGlobalInterface::getInstance().getOdomData(odomData);

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = this->get_clock()->now();
  
  msg.header.frame_id = ns+"/odom";
  msg.child_frame_id = ns+"/base_link";

  msg.pose.pose.position.x = odomData.pos_x; 
  msg.pose.pose.position.y = odomData.pos_y;
  msg.pose.pose.position.z = odomData.pos_z;
  msg.pose.pose.orientation.x = odomData.orient_x;
  msg.pose.pose.orientation.y = odomData.orient_y;
  msg.pose.pose.orientation.z = odomData.orient_z;
  msg.pose.pose.orientation.w = odomData.orient_w;

  msg.twist.twist.linear.x = odomData.twist_lin_x;
  msg.twist.twist.linear.y = odomData.twist_lin_y;
  msg.twist.twist.linear.z = odomData.twist_lin_z;
  msg.twist.twist.angular.x = odomData.twist_ang_x;
  msg.twist.twist.angular.y = odomData.twist_ang_y;
  msg.twist.twist.angular.z = odomData.twist_ang_z;

  for (int i=0; i<odomData.pose_cov.size(); i++) {
    msg.pose.covariance[i] = odomData.pose_cov.at(i);
    msg.twist.covariance[i] = odomData.twist_cov.at(i);
  }

  t265_publisher_->publish(msg);
}

void ShelfinoHWNode::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null odometry message");
    return;
  }

  try {
    nav_msgs::msg::Odometry new_msg;

    new_msg.header = msg->header;
    new_msg.pose = msg->pose;
    new_msg.child_frame_id = msg->child_frame_id;
    new_msg.twist = msg->twist;

    odom_publisher_->publish(*msg);
    handle_shelfino_pose(*msg);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error in odom_callback: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown error in odom_callback");
  }
}


/*void ShelfinoHWNode::odom_timer_callback()
{
  RobotStatus::OdometryData odomData; 
  HardwareGlobalInterface::getInstance().getOdomData(odomData);

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = this->get_clock()->now();

  msg.header.frame_id = ns+"/odom";
  msg.child_frame_id = ns+"/base_footprint";

  msg.pose.pose.position.x = odomData.pos_x; 
  msg.pose.pose.position.y = odomData.pos_y;
  msg.pose.pose.position.z = odomData.pos_z;
  msg.pose.pose.orientation.x = odomData.orient_x;
  msg.pose.pose.orientation.y = odomData.orient_y;
  msg.pose.pose.orientation.z = odomData.orient_z;
  msg.pose.pose.orientation.w = odomData.orient_w;

  if (msg.pose.pose.orientation.x == 0 && 
      msg.pose.pose.orientation.y == 0 &&
      msg.pose.pose.orientation.z == 0 &&
      msg.pose.pose.orientation.w == 0)
  {
    msg.pose.pose.orientation.w = 1;
  }

  msg.twist.twist.linear.x = odomData.twist_lin_x;
  msg.twist.twist.linear.y = odomData.twist_lin_y;
  msg.twist.twist.linear.z = odomData.twist_lin_z;
  msg.twist.twist.angular.x = odomData.twist_ang_x;
  msg.twist.twist.angular.y = odomData.twist_ang_y;
  msg.twist.twist.angular.z = odomData.twist_ang_z;

  for (int i=0; i<odomData.pose_cov.size(); i++) {
    msg.pose.covariance[i] = odomData.pose_cov.at(i);
    msg.twist.covariance[i] = odomData.twist_cov.at(i);
  }

  RCLCPP_INFO(this->get_logger(), "Publishing odom from encoders %s %s     %f %f %f -- %f %f %f %f",
    msg.header.frame_id.c_str(), msg.child_frame_id.c_str(), 
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, 
    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
  );

  odom_publisher_->publish(msg);
  handle_shelfino_pose(msg);
}*/

void ShelfinoHWNode::enc_callback()
{
  RobotStatus::HardwareData hwData; 
  HardwareGlobalInterface::getInstance().getHardwareData(hwData);

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->get_clock()->now();

  msg.header.frame_id = "";

  msg.name.push_back(this->ns+"/left_wheel_joint");
  msg.position.push_back(hwData.leftWheel.ticks);
  msg.velocity.push_back(hwData.leftWheel.omega);
  msg.name.push_back(this->ns+"/right_wheel_joint");
  msg.position.push_back(hwData.rightWheel.ticks);
  msg.velocity.push_back(hwData.rightWheel.omega);

  encoders_publisher_->publish(msg);
}


void ShelfinoHWNode::handle_shelfino_cmd_vel(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  double v = 0., omega = 0.;
  v = msg->linear.x;
  omega = msg->angular.z;

  if(shelfino_id == 1 || shelfino_id == 3){
    v = -msg->linear.x;
    omega = -msg->angular.z;
  }

  HardwareGlobalInterface::getInstance().vehicleMove(v,omega);

  return;
}


void ShelfinoHWNode::handle_shelfino_pose(nav_msgs::msg::Odometry msg)
{
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = msg.header.stamp;

  t.header.frame_id = ns+"/odom";
  t.child_frame_id = ns+"/base_link";

  t.transform.translation.x = msg.pose.pose.position.x;
  t.transform.translation.y = msg.pose.pose.position.y;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = msg.pose.pose.orientation.x;
  t.transform.rotation.y = msg.pose.pose.orientation.y;
  t.transform.rotation.z = msg.pose.pose.orientation.z;
  t.transform.rotation.w = msg.pose.pose.orientation.w;

  if (t.transform.rotation.x == 0 && 
      t.transform.rotation.y == 0 && 
      t.transform.rotation.z == 0 && 
      t.transform.rotation.w == 0)
  {
    t.transform.rotation.w = 1.0;
  }

  // Send the transformation
  tf_broadcaster_->sendTransform(t);
}

void ShelfinoHWNode::handle_power_srv(std_srvs::srv::SetBool::Request::SharedPtr request,
                      std_srvs::srv::SetBool::Response::SharedPtr response)
{
  RCLCPP_INFO(this->get_logger(), "Received request for power %s", std::to_string(request->data));
  if(request->data){
    HardwareGlobalInterface::getInstance().robotOnVelControl();
    response->message = ns+" motors powered on";
    RCLCPP_INFO(this->get_logger(), "motors powered on");
  } else {
    HardwareGlobalInterface::getInstance().robotOff();
    response->message = ns+" motors powered off";
    RCLCPP_INFO(this->get_logger(), "motors powered off");
  }
  response->success = true;
}

/**
 * @brief Main function that initialize the ROS node
 * 
 * @param argc Number of arguments passed from the command line
 * @param argv Array of arguments passed from the command line
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShelfinoHWNode>());

  rclcpp::shutdown();
  return 0;
}
