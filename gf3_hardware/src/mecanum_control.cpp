#include "gf3_hardware/mecanum_control.hpp"
namespace gf3_hardware
{
  const double WHEEL_RADIUS = 0.05;
  const double WHEEL_SEPARATION_WIDTH = 0.405;
  const double WHEEL_SEPARATION_LENGTH = 0.304;
  const double ODOM_LINEAR_CALIBRAION_FACTOR = 1.12;
  const double ODOM_ANGULAR_CALIBRAION_FACTOR = 2.1;
  const double ratio = 10.0;
  const uint32_t motor_id_[2] = {0x001U, 0x002U}; // (FL - 0x002, M1), (RL - 0x002, M2), (FR - 0x001, M1), (RR - 0x001, M2)
  const uint32_t motor_reply_id_[2] = {0x701U, 0x702U}; // (FL - 0x002, M1), (RL - 0x002, M2), (FR - 0x001, M1), (RR - 0x001, M2)

MecanumControl::MecanumControl()
:Node("MecanumControl"),
received_FL_w_(0), received_FR_w_(0), received_RL_w_(0), received_RR_w_(0),
is_FL_received_(false), is_FR_received_(false), is_RL_received_(false), is_RR_received_(false),
x_(0), y_(0), th_(0),
prev_time_(rclcpp::Clock(RCL_ROS_TIME).now().seconds())
{
  // init_motor();
  m1_command_.data[0] = 0xCF;
  m1_command_.data[1] = 0x01; // M1 enable
  m1_command_.data[4] = 0x01; // M2 enable
  m1_command_.data[7] = 0x07; // set reply type

  m1_command_.id = motor_id_[1];
  m1_command_.data[2] = 0x00; // for FL(M1)
  m1_command_.data[3] = 0x00;
  m1_command_.data[5] = 0x00; // for RL(M2)
  m1_command_.data[6] = 0x00;
  
  m2_command_.data[0] = 0xCF;
  m2_command_.data[1] = 0x01; // M1 enable
  m2_command_.data[4] = 0x01; // M2 enable
  m2_command_.data[7] = 0x07; // set reply type

  m2_command_.id = motor_id_[0];
  m2_command_.data[2] = 0x00; // for FR(M1)
  m2_command_.data[3] = 0x00;
  m2_command_.data[5] = 0x00; // for RR(M2)
  m2_command_.data[6] = 0x00;

  can_state_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_feedback", 1, std::bind(&MecanumControl::can_reply_callback, this, _1));
  cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&MecanumControl::cmd_vel_callback, this, _1));
  can_command_publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_command", 10);
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  
}

MecanumControl::~MecanumControl()
{
  
}

void MecanumControl::cmd_vel_callback(const geometry_msgs::msg::Twist &msg){
  
  // double wheel_front_left = (1/WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*msg.angular.z); 
  double wheel_front_left = (1/WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*msg.angular.z);
  double wheel_front_right = (1/WHEEL_RADIUS) * (msg.linear.x + msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*msg.angular.z);
  double wheel_rear_left = (1/WHEEL_RADIUS) * (msg.linear.x + msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*msg.angular.z);
  double wheel_rear_right = (1/WHEEL_RADIUS) * (msg.linear.x - msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*msg.angular.z);

  int16_t target_FL_rpm = wheel_front_left / (2 * M_PI) * 60 * ratio;
  int16_t target_FR_rpm = wheel_front_right / (2 * M_PI) * 60 * ratio;
  int16_t target_RL_rpm = wheel_rear_left / (2 * M_PI) * 60 * ratio;
  int16_t target_RR_rpm = wheel_rear_right / (2 * M_PI) * 60 * ratio;
  //   int16_t target_FL_rpm =10;
  // int16_t target_FR_rpm = 10;
  // int16_t target_RL_rpm = 10;
  // int16_t target_RR_rpm = 10;

    // FL, RL
  m1_command_.id = motor_id_[1];
  m1_command_.data[2] = target_FL_rpm & 0xFF; // for FL(M1)
  m1_command_.data[3] = (target_FL_rpm >> 8) & 0xFF;
  m1_command_.data[5] = target_RL_rpm & 0xFF; // for RL(M2)
  m1_command_.data[6] = (target_RL_rpm >> 8) & 0xFF;

  m2_command_.id = motor_id_[0];
  m2_command_.data[2] = target_FR_rpm & 0xFF; // for FR(M1)
  m2_command_.data[3] = (target_FR_rpm >> 8) & 0xFF;
  m2_command_.data[5] = target_RR_rpm & 0xFF; // for RR(M2)
  m2_command_.data[6] = (target_RR_rpm >> 8) & 0xFF;
  
}

void MecanumControl::can_reply_callback(const can_msgs::msg::Frame &msg)
{
  auto it = std::find(std::begin(motor_reply_id_), std::end(motor_reply_id_), msg.id);
  if (it != std::end(motor_reply_id_)) {
      int index = std::distance(motor_reply_id_, it);
      std::cout << "Reply from Index: " << index << "found" << std::endl;
  } else {
      RCLCPP_INFO(rclcpp::get_logger("MecanumControl"), "not found...");

    }
  // c4(196)-motor1, c9(201)-motor2 
  if (msg.id == 0x701U){ // Right controller/ (2 * M_PI) * 60 * ratio
    if (msg.data[0] == 0xC4){ // FR
      int16_t raw_rpm = ((msg.data[2] & 0xFF) +
                        ((msg.data[3] << 8) & 0xFF'00));
      received_FR_w_ = raw_rpm / 60 * (2* M_PI)  / ratio;
      is_FR_received_ = true;
      // RCLCPP_INFO(rclcpp::get_logger("MecanumControl"), "ID 1 - FR received: %f",received_FR_w_);
    }
    if (msg.data[0] == 0xC9){ // RR
      int16_t raw_rpm = ((msg.data[2] & 0xFF) +
                        ((msg.data[3] << 8) & 0xFF'00));
      received_RR_w_ = raw_rpm / 60 * (2* M_PI)  / ratio;
      is_RR_received_ = true;
      // RCLCPP_INFO(rclcpp::get_logger("MecanumControl"), "ID 1 - RR received: %f",received_RR_w_);
    }
  }
  else if(msg.id == 0x702U){ // Left controller
    if (msg.data[0] == 0xC4){ // FL
      int16_t raw_rpm = ((msg.data[2] & 0xFF) +
                        ((msg.data[3] << 8) & 0xFF'00));
      received_FL_w_ = raw_rpm / 60 * (2* M_PI)  / ratio;
      is_FL_received_ = true;
      // RCLCPP_INFO(rclcpp::get_logger("MecanumControl"), "ID 1 - FL received: %f",received_FR_w_);
    }
    if (msg.data[0] == 0xC9){ // RL
      int16_t raw_rpm = ((msg.data[2] & 0xFF) +
                        ((msg.data[3] << 8) & 0xFF'00));
      received_RL_w_ = raw_rpm / 60 * (2* M_PI)  / ratio;
      is_RL_received_ = true;
      // RCLCPP_INFO(rclcpp::get_logger("MecanumControl"), "ID 1 - RL received: %f",received_FR_w_);
    }
  }
}


int MecanumControl::MainLoop()
{
  // Publish the can command
  can_command_publisher_->publish(m1_command_);
  can_command_publisher_->publish(m2_command_);

  // Spin some nodes to check for wheel velocity updates
  rclcpp::spin_some(shared_from_this());

  // Check if wheel velocities are all received
  if (is_FL_received_ == true && is_FR_received_ == true &&is_RL_received_ == true &&is_RR_received_ == true){ // if All w are received
    // Calculate linear and angular velocities from wheel velocities
    double vx = (received_FR_w_ + received_FL_w_ + received_RL_w_ + received_RR_w_) * WHEEL_RADIUS / 4 * ODOM_LINEAR_CALIBRAION_FACTOR;
    double vy = (received_FR_w_ - received_FL_w_ + received_RL_w_ - received_RR_w_) * WHEEL_RADIUS / 4 * ODOM_LINEAR_CALIBRAION_FACTOR;
    double vth = (received_FR_w_ - received_FL_w_ - received_RL_w_ + received_RR_w_) * WHEEL_RADIUS / (4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)) * ODOM_ANGULAR_CALIBRAION_FACTOR;
    
    // Calculate the current time, then calculate the interval (dt) since last reading
    double current_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();;
    double dt = current_time - prev_time_;
    prev_time_ = current_time;
    // RCLCPP_INFO(rclcpp::get_logger("MecanumControl"), "dt calculated: %f",dt);

    // Calculate displacement in x, y & theta from linear & angular velocities respectively
    double delta_x = ((vx * cos(th_) - vy * sin(th_)) * dt);
    double delta_y = ((vx * sin(th_) + vy * cos(th_)) * dt);
    double delta_th = vth * dt;

    // Update position
    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // Set up the odometry message with updated values
    odom_.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    odom_.child_frame_id = "base_link";
    odom_.twist.twist.linear.x = vx;
    odom_.twist.twist.linear.y = vy;
    odom_.twist.twist.angular.z = vth;

    odom_.pose.pose.position.x = x_;
    odom_.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    odom_.pose.pose.orientation.x = q.x();
    odom_.pose.pose.orientation.y = q.y();
    odom_.pose.pose.orientation.z = q.z();
    odom_.pose.pose.orientation.w = q.w();

    // Publish the odometry message
    odom_publisher_->publish(odom_);
   
    // Create the transform message based on odometry message
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom_.pose.pose.orientation;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // Reset the received flags
    is_FR_received_ = false;
    is_FL_received_ = false;
    is_RL_received_ = false;
    is_RR_received_ = false;
  }
  return 0;
}
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Rate loop_rate(100);
  auto mecanum = std::make_shared<gf3_hardware::MecanumControl>();
  while(rclcpp::ok()){
    mecanum->MainLoop();
    // loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}