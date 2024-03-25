#include <uav_simulator/quadcopterPlugin.h>


#include <cmath>
#include <stdlib.h>
#include <iostream>


namespace gazebo {

DroneSimpleController::DroneSimpleController()
{ 
  navi_state = LANDED_MODEL;
  m_posCtrl = false;
  m_velMode = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DroneSimpleController::~DroneSimpleController()
{
  // Deprecated since Gazebo 8.
  //event::Events::DisconnectWorldUpdateBegin(updateConnection);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DroneSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  if(!ros::isInitialized()){
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package)");
  }
  
  world = _model->GetWorld();
  ROS_INFO("The drone plugin is loading!");
  
  //load parameters
  cmd_normal_topic_ = "/CERLAB/quadcopter/cmd_vel";
  cmd_acc_topic_ = "/CERLAB/quadcopter/cmd_acc";
  target_pose_topic_ = "/CERLAB/quadcopter/setpoint_pose";
  takeoff_topic_ = "/CERLAB/quadcopter/takeoff";
  land_topic_ = "/CERLAB/quadcopter/land";
  reset_topic_ = "/CERLAB/quadcopter/reset";
  posctrl_topic_ = "/CERLAB/quadcopter/posctrl";
  gt_pose_topic_ = "/CERLAB/quadcopter/pose_raw";
  gt_vel_topic_ = "/CERLAB/quadcopter/vel_raw";
  gt_acc_topic_ = "/CERLAB/quadcopter/acc_raw";
  gt_odom_topic_ = "/CERLAB/quadcopter/odom_raw";
  switch_mode_topic_ = "/CERLAB/quadcopter/vel_mode";
  // imu_topic_ = "/CERLAB/quadcopter/imu";
  
  
  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = boost::dynamic_pointer_cast<physics::Link>(world->EntityByName(link_name_));
  }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("maxForce"))
    max_force_ = -1;
  else
    max_force_ = _sdf->GetElement("maxForce")->Get<double>();


  if (!_sdf->HasElement("motionSmallNoise"))
    motion_small_noise_ = 0;
  else
    motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoise"))
    motion_drift_noise_ = 0;
  else
    motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoiseTime"))
    motion_drift_noise_time_ = 1.0;
  else
    motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();


  // get inertia and mass of quadrotor body
  inertia = link->GetInertial()->PrincipalMoments();
  mass = link->GetInertial()->Mass();

  node_handle_ = new ros::NodeHandle;
  

  // subscribe command: control command
  if (!cmd_normal_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::TwistStamped>(
      cmd_normal_topic_, 1,
      boost::bind(&DroneSimpleController::CmdCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    cmd_subscriber_ = node_handle_->subscribe(ops);
    
    if( cmd_subscriber_.getTopic() != "")
        ROS_INFO_NAMED("quadrotor_simple_controller", "Using cmd_topic %s.", cmd_normal_topic_.c_str());
    else
        ROS_INFO("cannot find the command topic!");
  }

  // subscrbe command: acceleratioin command
  if (!cmd_acc_topic_.empty()){
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<mavros_msgs::PositionTarget>(
      cmd_acc_topic_, 1,
      boost::bind(&DroneSimpleController::CmdAccCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    cmd_acc_subscriber_ = node_handle_->subscribe(ops);
    
    if( cmd_acc_subscriber_.getTopic() != "")
        ROS_INFO_NAMED("quadrotor_acceleraton_controller", "Using cmd_acc_topic %s.", cmd_acc_topic_.c_str());
    else
        ROS_INFO("cannot find the command topic!");    
  }

  if (!target_pose_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
      target_pose_topic_, 1,
      boost::bind(&DroneSimpleController::TargetPoseCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    target_pose_subscriber_ = node_handle_->subscribe(ops);
    if (cmd_subscriber_.getTopic() != ""){
        ROS_INFO_NAMED("quadrotor_simple_controller", "Using target pose topic %s.", target_pose_topic_.c_str());
    }
    else{
        ROS_INFO("cannot find the command topic!");
    }
  }
  
  if (!posctrl_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
      posctrl_topic_, 1,
      boost::bind(&DroneSimpleController::PosCtrlCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    posctrl_subscriber_ = node_handle_->subscribe(ops);
    
    if( posctrl_subscriber_.getTopic() != "")
        ROS_INFO("find the position control topic!");
    else
        ROS_INFO("cannot find the position control topic!");
  }
  

  // subscribe imu
  // if (!imu_topic_.empty())
  // {
  //   ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
  //     imu_topic_, 1,
  //     boost::bind(&DroneSimpleController::ImuCallback, this, _1),
  //     ros::VoidPtr(), &callback_queue_);
  //   imu_subscriber_ = node_handle_->subscribe(ops);
    
  //   if(imu_subscriber_.getTopic() !="")
  //       ROS_INFO_NAMED("quadrotor_simple_controller", "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
  //   else
  //       ROS_INFO("cannot find the IMU topic!");
  // }

  // subscribe command: take off command
  if (!takeoff_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      takeoff_topic_, 1,
      boost::bind(&DroneSimpleController::TakeoffCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    takeoff_subscriber_ = node_handle_->subscribe(ops);
    if( takeoff_subscriber_.getTopic() != "")
        ROS_INFO("find the takeoff topic");
    else
        ROS_INFO("cannot find the takeoff topic!");
  }

  // subscribe command: land command
  if (!land_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      land_topic_, 1,
      boost::bind(&DroneSimpleController::LandCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    land_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe command: reset command
  if (!reset_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      reset_topic_, 1,
      boost::bind(&DroneSimpleController::ResetCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    reset_subscriber_ = node_handle_->subscribe(ops);
  }
  
  if (!gt_pose_topic_.empty()){
      pub_gt_pose_ = node_handle_->advertise<geometry_msgs::PoseStamped>(gt_pose_topic_,1024);    
  }
  
  if (!gt_vel_topic_.empty()){
      pub_gt_vec_ = node_handle_->advertise<geometry_msgs::TwistStamped>(gt_vel_topic_, 1024);
  }

  if (!gt_vel_topic_.empty()){
      pub_gt_acc_ = node_handle_->advertise<geometry_msgs::TwistStamped>(gt_acc_topic_, 1024);
  }

  if (!gt_odom_topic_.empty()){
      pub_gt_odom_ = node_handle_->advertise<nav_msgs::Odometry>(gt_odom_topic_, 1024);
  }
  
  if (!switch_mode_topic_.empty()){
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
        switch_mode_topic_, 1,
        boost::bind(&DroneSimpleController::SwitchModeCallback, this, _1),
        ros::VoidPtr(), &callback_queue_);
      switch_mode_subscriber_ = node_handle_->subscribe(ops);
  }
      

  LoadControllerSettings(_model, _sdf);
  
  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DroneSimpleController::Update, this));
}

void DroneSimpleController::LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    controllers_.roll.Load(_sdf, "rollpitch");
    controllers_.pitch.Load(_sdf, "rollpitch");
    controllers_.yaw.Load(_sdf, "yaw");
    bool is_yaw = true;
    controllers_.yaw_angle.Load(_sdf, "yawAngle", is_yaw);
    controllers_.velocity_x.Load(_sdf, "velocityXY");
    controllers_.velocity_y.Load(_sdf, "velocityXY");
    controllers_.velocity_z.Load(_sdf, "velocityZ");
    
    controllers_.pos_x.Load(_sdf, "positionXY");
    controllers_.pos_y.Load(_sdf, "positionXY");
    controllers_.pos_z.Load(_sdf, "positionZ");

    controllers_.acc_x.Load(_sdf, "accelerationXY");
    controllers_.acc_y.Load(_sdf, "accelerationXY");
    controllers_.acc_z.Load(_sdf, "accelerationZ");
    
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void DroneSimpleController::CmdCallback(const geometry_msgs::TwistStampedConstPtr& cmd)
{
  cmd_val = cmd->twist;
  m_posCtrl = false;
  acc_control =  false;
}

void DroneSimpleController::CmdAccCallback(const mavros_msgs::PositionTargetConstPtr& cmd_acc_msg)
{
  cmd_acc = *cmd_acc_msg;
  m_posCtrl = false;
  acc_control = true;
}

void DroneSimpleController::TargetPoseCallback(const geometry_msgs::PoseStampedConstPtr& setpoint){
  pose_setpoint = setpoint->pose;
  m_posCtrl = true;
  acc_control =  false;
  navi_state = FLYING_MODEL;
}

void DroneSimpleController::PosCtrlCallback(const std_msgs::BoolConstPtr& cmd){
    m_posCtrl = cmd->data;
}

void DroneSimpleController::ImuCallback(const sensor_msgs::ImuConstPtr& imu)
{
  //directly read the quternion from the IMU data
  pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.Rot().Euler();
  angular_velocity = pose.Rot().RotateVector(ignition::math::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
}

void DroneSimpleController::TakeoffCallback(const std_msgs::EmptyConstPtr& msg)
{
  if(navi_state == LANDED_MODEL)
  {
    navi_state = TAKINGOFF_MODEL;
    m_timeAfterCmd = 0;
    ROS_INFO("%s","\nQuadrotor takes off!!");
  }
}

void DroneSimpleController::LandCallback(const std_msgs::EmptyConstPtr& msg)
{
  if(navi_state == FLYING_MODEL||navi_state == TAKINGOFF_MODEL)
  {
    navi_state = LANDING_MODEL;
    m_timeAfterCmd = 0;
    ROS_INFO("%s","\nQuadrotor lands!!");
  }

}

void DroneSimpleController::ResetCallback(const std_msgs::EmptyConstPtr& msg)
{
  ROS_INFO("%s","\nReset quadrotor!!");
}

void DroneSimpleController::SwitchModeCallback(const std_msgs::BoolConstPtr& msg){
    m_velMode = msg->data;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void DroneSimpleController::Update()
{
    
    // Get new commands/state
    callback_queue_.callAvailable();
  
    // Get simulator time
    common::Time sim_time = world->SimTime();
    double dt = (sim_time - last_time).Double();
    if (dt == 0.0) return;
    
    UpdateState(dt);
    UpdateDynamics(dt);
    
    // save last time stamp
    last_time = sim_time;   
}

void DroneSimpleController::UpdateState(double dt){
    if(navi_state == TAKINGOFF_MODEL){
        m_timeAfterCmd += dt;
        if (m_timeAfterCmd > 0.5){
            navi_state = FLYING_MODEL;
            std::cout << "Entering flying model!" << std::endl;
        }
    }else if(navi_state == LANDING_MODEL){
        m_timeAfterCmd += dt;
        if(m_timeAfterCmd > 1.0){
            navi_state = LANDED_MODEL;
            std::cout << "Landed!" <<std::endl;
        }
    }else
        m_timeAfterCmd = 0;
}


void DroneSimpleController::UpdateDynamics(double dt){
    ignition::math::Vector3d force, torque;
   
    // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  //  if (imu_subscriber_.getTopic()=="")
    {
      pose = link->WorldPose();
      angular_velocity = link->WorldAngularVel();
      euler = pose.Rot().Euler();
    }
   // if (state_topic_.empty())
    {
      acceleration = (link->WorldLinearVel() - velocity) / dt;
      velocity = link->WorldLinearVel();
    }
    
    
    //publish the ground truth pose of the drone to the ROS topic
    geometry_msgs::PoseStamped gt_pose_stamped;
    geometry_msgs::Pose gt_pose;
    gt_pose.position.x = pose.Pos().X();
    gt_pose.position.y = pose.Pos().Y();
    gt_pose.position.z = pose.Pos().Z();
    
    gt_pose.orientation.w = pose.Rot().W();
    gt_pose.orientation.x = pose.Rot().X();
    gt_pose.orientation.y = pose.Rot().Y();
    gt_pose.orientation.z = pose.Rot().Z();

    gt_pose_stamped.pose = gt_pose;
    gt_pose_stamped.header.frame_id = "map";
    gt_pose_stamped.header.stamp = ros::Time::now();
    pub_gt_pose_.publish(gt_pose_stamped);
    
    //convert the acceleration and velocity into the body frame
    ignition::math::Vector3d body_vel = pose.Rot().RotateVectorReverse(velocity);
    ignition::math::Vector3d body_angular_vel = pose.Rot().RotateVectorReverse(angular_velocity);
    ignition::math::Vector3d body_acc = pose.Rot().RotateVectorReverse(acceleration);
    
    //publish the velocity
    geometry_msgs::TwistStamped tw_stamped;
    geometry_msgs::Twist tw;
    tw.linear.x = body_vel.X();
    tw.linear.y = body_vel.Y();
    tw.linear.z = body_vel.Z();
    tw.angular.x = body_angular_vel.X();
    tw.angular.y = body_angular_vel.Y();
    tw.angular.z = body_angular_vel.Z();
    tw_stamped.twist = tw;
    tw_stamped.header.frame_id = "base_link";
    tw_stamped.header.stamp = ros::Time::now();
    pub_gt_vec_.publish(tw_stamped);
    
    //publish the acceleration
    geometry_msgs::TwistStamped tw_acc_stamped;
    geometry_msgs::Twist tw_acc;
    tw_acc.linear.x = body_acc.X();
    tw_acc.linear.y = body_acc.Y();
    tw_acc.linear.z = body_acc.Z();
    tw_acc_stamped.twist = tw_acc;
    tw_acc_stamped.header.frame_id = "base_link";
    tw_acc_stamped.header.stamp = ros::Time::now();
    pub_gt_acc_.publish(tw_acc_stamped);

    //publish odometry
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose = gt_pose;
    odom.twist.twist = tw;
    pub_gt_odom_.publish(odom);
    
            
    ignition::math::Vector3d poschange = pose.Pos() - position;
    position = pose.Pos();
    
    ignition::math::Vector3d accchange = body_acc - prev_acceleration;
    prev_acceleration = body_acc;


    // Get gravity
    ignition::math::Vector3d gravity_body = pose.Rot().RotateVector(world->Gravity());
    double gravity = gravity_body.Length();
    double load_factor = gravity * gravity / world->Gravity().Dot(gravity_body);  // Get gravity
  
    // Rotate vectors to coordinate frames relevant for control
    ignition::math::Quaterniond heading_quaternion(cos(euler.Z()/2),0,0,sin(euler.Z()/2));
    ignition::math::Vector3d velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
    ignition::math::Vector3d acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
    ignition::math::Vector3d angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);
  
    // update controllers
    force.Set(0.0, 0.0, 0.0); 
    torque.Set(0.0, 0.0, 0.0);
    
    if (acc_control){
        // yaw control seperately
        double yawAngleSetpoint = cmd_acc.yaw;
        // if (isnan(cmd_acc.yaw)) 
        //   std::cout << "input nan" << std::endl;
        // if (isnan(yawAngleSetpoint)) 
        //   std::cout << "input nan2" << std::endl;
        double yaw_rate = controllers_.yaw_angle.update(yawAngleSetpoint, euler.Z(), yawAngleSetpoint - euler.Z(), dt);
        

        // ignition::math::Vector3d desired_acc (cmd_acc.acceleration_or_force.x, cmd_acc.acceleration_or_force.y, cmd_acc.acceleration_or_force.z+9.8);

        // double yaw = euler.Z();
        // ignition::math::Vector3d direction (cos(yaw), sin(yaw), 0.0);
        // ignition::math::Vector3d zDirection = desired_acc/desired_acc.Length();
        // ignition::math::Vector3d yDirection = zDirection.Cross(direction)/(zDirection.Cross(direction)).Length();
        // ignition::math::Vector3d xDirection = yDirection.Cross(zDirection)/(yDirection.Cross(zDirection)).Length();

        // Eigen::Matrix3d attitudeRefRot;
        // attitudeRefRot << xDirection[0], yDirection[0], zDirection[0],
        //         xDirection[1], yDirection[1], zDirection[1],
        //         xDirection[2], yDirection[2], zDirection[2];


        // Eigen::Vector4d attitudeRefQuat = rot2Quaternion(attitudeRefRot);
        // geometry_msgs::Quaternion quatMsg;
        // quatMsg.w = attitudeRefQuat(0);
        // quatMsg.x = attitudeRefQuat(1);
        // quatMsg.y = attitudeRefQuat(2);
        // quatMsg.z = attitudeRefQuat(3);

        // double roll_command, pitch_command, yaw_command;
        // rpy_from_quaternion(quatMsg, roll_command, pitch_command, yaw_command);

        // torque.X() = inertia.X() *  controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
        // torque.Y() = inertia.Y() *  controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);            
        // torque.Z() = inertia.Z() *  controllers_.yaw.update(yaw_rate, angular_velocity.Z(), 0, dt);
        // force.Z()  = mass * desired_acc.Length();
        // // std::cout << "current yaw: " << yaw << " target yaw: " << yawAngleSetpoint << std::endl;
        // // std::cout << "target yaw rate: " << yaw_rate << std::endl;
        // // std::cout << "torque z: " << torque.Z() << std::endl;



        ignition::math::Vector3d desired_acc (cmd_acc.acceleration_or_force.x, cmd_acc.acceleration_or_force.y, cmd_acc.acceleration_or_force.z);
        ignition::math::Vector3d desired_body_acc = pose.Rot().RotateVectorReverse(desired_acc);
        double pitch_command =  controllers_.acc_x.update(desired_body_acc.X(), body_acc.X(), accchange.X(), dt) / gravity;
        double roll_command  = -controllers_.acc_y.update(desired_body_acc.Y(), body_acc.Y(), accchange.Y(), dt) / gravity;
        torque.X() = inertia.X() *  controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
        torque.Y() = inertia.Y() *  controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);            
        force.Z()  = mass      * (controllers_.acc_z.update(desired_body_acc.Z(),  body_acc.Z(), accchange.Z(), dt) + load_factor * gravity);
        torque.Z() = inertia.Z() *  controllers_.yaw.update(yaw_rate, angular_velocity_body.Z(), 0, dt);

        // cout << "desired acc x: " << desired_body_acc.X() << " body acc x: " << body_acc.X() << " acc chagne: " << accchange.X() << endl;
        // cout << "pitch command: " << pitch_command << " degree: " << pitch_command * 57.3 << endl;
        if (isnan(torque.Z())){
          torque.Z() = 0.0; // this means yaw angle target is not valid
        }
        // cout << "yaw angle set: " << yawAngleSetpoint << "current yaw: " << euler.Z()  << " yaw rate: " << yaw_rate << " torque z: " << torque.Z() << endl; 
    }
    else{
      if( m_posCtrl){
          //position control
          if(navi_state == FLYING_MODEL){
              // double vx = controllers_.pos_x.update(cmd_val.linear.x, position.X(), poschange.X(), dt);
              // double vy = controllers_.pos_y.update(cmd_val.linear.y, position.Y(), poschange.Y(), dt);
              // double vz = controllers_.pos_z.update(cmd_val.linear.z, position.Z(), poschange.Z(), dt);
              // double yaw_rate = controllers_.yaw_angle.update(cmd_val.angular.z, euler.Z(), cmd_val.angular.z - euler.Z(), dt);

              double vx = controllers_.pos_x.update(pose_setpoint.position.x, position.X(), poschange.X(), dt);
              double vy = controllers_.pos_y.update(pose_setpoint.position.y, position.Y(), poschange.Y(), dt);
              double vz = controllers_.pos_z.update(pose_setpoint.position.z, position.Z(), poschange.Z(), dt);
              double yawAngleSetpoint = rpy_from_quaternion(pose_setpoint.orientation);
              double yaw_rate = controllers_.yaw_angle.update(yawAngleSetpoint, euler.Z(), yawAngleSetpoint - euler.Z(), dt);

              ignition::math::Vector3d vb = heading_quaternion.RotateVectorReverse(ignition::math::Vector3d(vx,vy,vz));
              double pitch_command =  controllers_.velocity_x.update(vb.X(), velocity_xy.X(), acceleration_xy.X(), dt) / gravity;
              double roll_command  = -controllers_.velocity_y.update(vb.Y(), velocity_xy.Y(), acceleration_xy.Y(), dt) / gravity;
              torque.X() = inertia.X() *  controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
              torque.Y() = inertia.Y() *  controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);            
              force.Z()  = mass      * (controllers_.velocity_z.update(vz,  velocity.Z(), acceleration.Z(), dt) + load_factor * gravity);
              torque.Z() = inertia.Z() *  controllers_.yaw.update(yaw_rate, angular_velocity_body.Z(), 0, dt);
              if (isnan(torque.Z())){
                torque.Z() = 0.0; // this means yaw angle target is not valid
              }
          }
      }else{
          //normal control
          if( navi_state == FLYING_MODEL )//&& cmd_val.linear.x >= 0 && cmd_val.linear.y >= 0)
          {
            //hovering
            double pitch_command =  controllers_.velocity_x.update(cmd_val.linear.x, velocity_xy.X(), acceleration_xy.X(), dt) / gravity;
            double roll_command  = -controllers_.velocity_y.update(cmd_val.linear.y, velocity_xy.Y(), acceleration_xy.Y(), dt) / gravity;
            torque.X() = inertia.X() *  controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
            torque.Y() = inertia.Y() *  controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
          }else{
            //control by velocity
            if( m_velMode){
                double pitch_command =  controllers_.velocity_x.update(cmd_val.angular.x, velocity_xy.X(), velocity_xy.X(), dt) / gravity;
                double roll_command  = -controllers_.velocity_y.update(cmd_val.angular.y, velocity_xy.Y(), velocity_xy.Y(), dt) / gravity;
                torque.X() = inertia.X() *  controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
                torque.Y() = inertia.Y() *  controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
            }else{
              //control by tilting
              torque.X() = inertia.X() *  controllers_.roll.update(cmd_val.angular.x, euler.X(), angular_velocity_body.X(), dt);
              torque.Y() = inertia.Y() *  controllers_.pitch.update(cmd_val.angular.y, euler.Y(), angular_velocity_body.Y(), dt);
            }

          }
          torque.Z() = inertia.Z() *  controllers_.yaw.update(cmd_val.angular.z, angular_velocity_body.Z(), 0, dt);
          force.Z()  = mass      * (controllers_.velocity_z.update(cmd_val.linear.z,  velocity.Z(), acceleration.Z(), dt) + load_factor * gravity);
      }
    }

    if (max_force_ > 0.0 && force.Z() > max_force_) force.Z() = max_force_;
    if (force.Z() < 0.0) force.Z() = 0.0;
    // process robot state information
    if (navi_state == FLYING_MODEL) {
      link->AddRelativeForce(force);
      link->AddRelativeTorque(torque);

    } else if (navi_state == TAKINGOFF_MODEL) {
      link->AddRelativeForce(force * 1.5);
      link->AddRelativeTorque(torque * 1.5);
    } else if (navi_state == LANDING_MODEL) {
      link->AddRelativeForce(force * 0.8);
      link->AddRelativeTorque(torque * 0.8);
    }
}
////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void DroneSimpleController::Reset()
{
  controllers_.roll.reset();
  controllers_.pitch.reset();
  controllers_.yaw.reset();
  controllers_.yaw_angle.reset();
  controllers_.velocity_x.reset();
  controllers_.velocity_y.reset();
  controllers_.velocity_z.reset();
  controllers_.pos_x.reset();
  controllers_.pos_y.reset();
  controllers_.pos_z.reset();
  controllers_.acc_x.reset();
  controllers_.acc_y.reset();
  controllers_.acc_z.reset();

  link->SetForce(ignition::math::Vector3d(0,0,0));
  link->SetTorque(ignition::math::Vector3d(0,0,0));

  // reset state
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  state_stamp = ros::Time();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DroneSimpleController)

} // namespace gazebo
