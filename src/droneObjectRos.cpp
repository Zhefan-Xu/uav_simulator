#include <uav_simulator/droneObjectRos.h>

void DroneObjectROS::initROSVars(ros::NodeHandle &node) {
  isFlying = false;
  isPosctrl = false;
  isVelMode = false;
  pubTakeOff = node.advertise<std_msgs::Empty>("/CERLAB/quadcopter/takeoff", 1024);
  pubLand = node.advertise<std_msgs::Empty>("/CERLAB/quadcopter/land", 1024);
  pubReset = node.advertise<std_msgs::Empty>("/CERLAB/quadcopter/reset", 1024);
  pubPosCtrl = node.advertise<std_msgs::Bool>("/CERLAB/quadcopter/posctrl", 1024);
  pubCmd = node.advertise<geometry_msgs::TwistStamped>("/CERLAB/quadcopter/cmd_vel", 1024);
  pubVelMode = node.advertise<std_msgs::Bool>("/CERLAB/quadcopter/vel_mode", 1024);
}

bool DroneObjectROS::takeOff() {
  if (isFlying)
    return false;

  pubTakeOff.publish(std_msgs::Empty());
  ROS_INFO("Taking Off...");

  isFlying = true;
  return true;
}

bool DroneObjectROS::land() {
  if (!isFlying)
    return false;

  pubLand.publish(std_msgs::Empty());
  ROS_INFO("Landing...");

  isFlying = false;
  return true;
}

bool DroneObjectROS::hover() {
  if (!isFlying)
    return false;

  twist_msg.header.frame_id = "base_link";
  twist_msg.header.stamp = ros::Time::now();

  twist_msg.twist.linear.x = 0;
  twist_msg.twist.linear.y = 0;
  twist_msg.twist.linear.z = 0;
  twist_msg.twist.angular.x = 0.0;
  twist_msg.twist.angular.y = 0.0;
  twist_msg.twist.angular.z = 0.0;

  pubCmd.publish(twist_msg);
  ROS_INFO("Hovering...");
  return true;
}

void DroneObjectROS::posCtrl(bool on) {
  isPosctrl = on;
  std_msgs::Bool bool_msg;
  bool_msg.data = on ? 1 : 0;
  pubPosCtrl.publish(bool_msg);
  if (on)
    ROS_INFO("Switching position control on...");
  else
    ROS_INFO("Switching position control off...");
}

void DroneObjectROS::velMode(bool on) {
  isVelMode = on;
  std_msgs::Bool bool_msg;
  bool_msg.data = on ? 1 : 0;
  pubVelMode.publish(bool_msg);
  if (on)
    ROS_INFO("Switching velocity mode on...");
  else
    ROS_INFO("Switching velocity mode off...");
}

bool DroneObjectROS::move(float lr, float fb, float ud, float w) {
  if (!isFlying)
    return false;

  twist_msg.header.frame_id = "base_link";
  twist_msg.header.stamp = ros::Time::now();

  twist_msg.twist.linear.x = 1.0;
  twist_msg.twist.linear.y = 1.0;
  twist_msg.twist.linear.z = ud;
  twist_msg.twist.angular.x = lr;
  twist_msg.twist.angular.y = fb;
  twist_msg.twist.angular.z = w;
  pubCmd.publish(twist_msg);
  ROS_INFO("Moving...");
  return true;
}

bool DroneObjectROS::moveTo(float x, float y, float z) {
  if (!isFlying)
    return false;
  twist_msg.header.frame_id = "base_link";
  twist_msg.header.stamp = ros::Time::now();
  twist_msg.twist.linear.x = x;
  twist_msg.twist.linear.y = y;
  twist_msg.twist.linear.z = z;
  // twist_msg.twist.angular.x = 0;
  // twist_msg.twist.angular.y = 0;
  // twist_msg.twist.angular.z = 0;

  pubCmd.publish(twist_msg);
  ROS_INFO("Moving...");
  return true;
}

bool DroneObjectROS::pitch(float speed) {
  if (!isFlying)
    return false;
  twist_msg.header.frame_id = "base_link";
  twist_msg.header.stamp = ros::Time::now();
  twist_msg.twist.linear.x = speed;
  // twist_msg.twist.linear.y = 0.0;
  // twist_msg.twist.linear.z = 0;
  pubCmd.publish(twist_msg);
  ROS_INFO("Pitching...");
  return true;
}
bool DroneObjectROS::roll(float speed) {
  if (!isFlying)
    return false;
  twist_msg.header.frame_id = "base_link";
  twist_msg.header.stamp = ros::Time::now();
  // twist_msg.twist.linear.x = 0.0;
  twist_msg.twist.linear.y = speed;
  // twist_msg.twist.linear.z = 0;
  pubCmd.publish(twist_msg);
  ROS_INFO("Rolling...");
  return true;
}

bool DroneObjectROS::rise(float speed) {
  if (!isFlying)
    return false;
  twist_msg.header.frame_id = "base_link";
  twist_msg.header.stamp = ros::Time::now();
  // twist_msg.twist.linear.x = 0.0;
  // twist_msg.twist.linear.y = 0.0;
  twist_msg.twist.linear.z = speed;
  // twist_msg.twist.angular.x = 0.0; // flag for preventing hovering
  // twist_msg.twist.angular.y = 0.0;
  // twist_msg.twist.angular.z = 0.0;
  pubCmd.publish(twist_msg);
  ROS_INFO("Rising...");
  return true;
}

bool DroneObjectROS::yaw(float speed) {
  if (!isFlying)
    return true;
  twist_msg.header.frame_id = "base_link";
  twist_msg.header.stamp = ros::Time::now();
  // twist_msg.twist.linear.x = 0.0;
  // twist_msg.twist.linear.y = 0.0;
  // twist_msg.twist.linear.z = 0;
  // twist_msg.twist.angular.x = 0.0; // flag for preventing hovering
  // twist_msg.twist.angular.y = 0.0;
  twist_msg.twist.angular.z = speed;
  pubCmd.publish(twist_msg);
  ROS_INFO("Turning head...");
  return true;
}
