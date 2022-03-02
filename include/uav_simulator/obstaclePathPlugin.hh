#ifndef OBSTACLEPATHPLUGIN_HH
#define OBSTACLEPATHPLUGIN_HH
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class DynamicObstacle : public ModelPlugin{
  private:
  	// Gazebo stuff
  	physics::ModelPtr model; // Pointer to the model
  	event::ConnectionPtr updateConnection; // Pointer to the update event connection

  	// custom attr
  	sdf::ElementPtr sdf;
  	double velocity;
  	double angularVelocity;
  	std::vector<ignition::math::Vector3d> path;
  	std::vector<std::vector<double>> pathWithAngle;
  	std::vector<int> timeKnot;

  public: 
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DynamicObstacle)
}

#endif