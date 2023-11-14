#ifndef ARDRONE_SIMPLE_CONTROL_H
#define ARDRONE_SIMPLE_CONTROL_H

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
// #include "ignition/math6/ignition/math.hh"
#include <ignition/math.hh>
#include <Eigen/Dense>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>

#include <uav_simulator/pidController.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#define LANDED_MODEL        0
#define FLYING_MODEL        1
#define TAKINGOFF_MODEL     2
#define LANDING_MODEL       3
using std::cout; using std::endl;

#define EPS 1E-6
namespace gazebo
{
class DroneSimpleController : public ModelPlugin
{
public:
  DroneSimpleController();
  virtual ~DroneSimpleController();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  void UpdateDynamics(double dt);
  void UpdateState(double dt);
  virtual void Reset();

private:
  double m_timeAfterCmd;
  bool m_posCtrl;
  bool m_velMode;
  bool acc_control =  false;
  unsigned int navi_state;
  
  
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  ros::Subscriber cmd_subscriber_;
  ros::Subscriber cmd_acc_subscriber_;
  ros::Subscriber acc_subscriber_;
  ros::Subscriber posctrl_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber target_pose_subscriber_;
  
  // extra robot control command
  ros::Subscriber takeoff_subscriber_;
  ros::Subscriber land_subscriber_;
  ros::Subscriber reset_subscriber_;
  ros::Subscriber switch_mode_subscriber_;
  
  ros::Publisher pub_gt_pose_;   //for publishing ground truth pose
  ros::Publisher pub_gt_vec_;   //ground truth velocity in the body frame
  ros::Publisher pub_gt_acc_;   //ground truth acceleration in the body frame
  ros::Publisher pub_gt_odom_; // publish odometry


  geometry_msgs::Twist cmd_val;
  mavros_msgs::PositionTarget cmd_acc;
  geometry_msgs::Pose pose_setpoint;
  // callback functions for subscribers
  void CmdCallback(const geometry_msgs::TwistStampedConstPtr&);
  void CmdAccCallback(const mavros_msgs::PositionTargetConstPtr&);
  void TargetPoseCallback(const geometry_msgs::PoseStampedConstPtr&);
  void PosCtrlCallback(const std_msgs::BoolConstPtr&);
  void ImuCallback(const sensor_msgs::ImuConstPtr&);
  void TakeoffCallback(const std_msgs::EmptyConstPtr&);
  void LandCallback(const std_msgs::EmptyConstPtr&);
  void ResetCallback(const std_msgs::EmptyConstPtr&);
  void SwitchModeCallback(const std_msgs::BoolConstPtr&);
 
 
  ros::Time state_stamp;
  ignition::math::Pose3d pose;
  ignition::math::Vector3d euler, velocity, acceleration, angular_velocity, position, prev_acceleration;

  std::string link_name_;
  std::string cmd_normal_topic_;
  std::string cmd_acc_topic_;
  std::string target_pose_topic_;
  std::string switch_mode_topic_;
  std::string posctrl_topic_;
  std::string imu_topic_;
  std::string takeoff_topic_;
  std::string land_topic_;
  std::string reset_topic_;
  std::string gt_pose_topic_;    //ground truth
  std::string gt_vel_topic_;
  std::string gt_acc_topic_;
  std::string gt_odom_topic_;
  
  double max_force_;
  double motion_small_noise_;
  double motion_drift_noise_;
  double motion_drift_noise_time_;

  struct Controllers {
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController yaw_angle;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
    PIDController pos_x;
    PIDController pos_y;
    PIDController pos_z;
    PIDController acc_x;
    PIDController acc_y;
    PIDController acc_z;
  } controllers_;

  ignition::math::Vector3d inertia;
  double mass;

  /// brief save last_time
  common::Time last_time;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

inline geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
  if (yaw > M_PI){
    yaw = yaw - 2*M_PI;
  }
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    return quaternion;
}

inline double rpy_from_quaternion(const geometry_msgs::Quaternion& quat){
  // return is [0, 2pi]
  tf2::Quaternion tf_quat;
  tf2::convert(quat, tf_quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

inline void rpy_from_quaternion(const geometry_msgs::Quaternion& quat, double &roll, double &pitch, double &yaw){
  tf2::Quaternion tf_quat;
  tf2::convert(quat, tf_quat);
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
}

inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R) {
    Eigen::Vector4d quat;
    double tr = R.trace();
    if (tr > 0.0){
        double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
        quat(0) = 0.25 * S;
        quat(1) = (R(2, 1) - R(1, 2)) / S;
        quat(2) = (R(0, 2) - R(2, 0)) / S;
        quat(3) = (R(1, 0) - R(0, 1)) / S;
    } 
    else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))){
        double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
        quat(0) = (R(2, 1) - R(1, 2)) / S;
        quat(1) = 0.25 * S;
        quat(2) = (R(0, 1) + R(1, 0)) / S;
        quat(3) = (R(0, 2) + R(2, 0)) / S;
    } 
    else if (R(1, 1) > R(2, 2)){
        double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
        quat(0) = (R(0, 2) - R(2, 0)) / S;
        quat(1) = (R(0, 1) + R(1, 0)) / S;
        quat(2) = 0.25 * S;
        quat(3) = (R(1, 2) + R(2, 1)) / S;
    } 
    else{
        double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
        quat(0) = (R(1, 0) - R(0, 1)) / S;
        quat(1) = (R(0, 2) + R(2, 0)) / S;
        quat(2) = (R(1, 2) + R(2, 1)) / S;
        quat(3) = 0.25 * S;
    }
    return quat;
}

}

#endif // ARDRONE_SIMPLE_CONTROL_H
