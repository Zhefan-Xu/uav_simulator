#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H


#include <gazebo/gazebo.hh>
#include <math.h>

class PIDController {
public:
  PIDController();
  virtual ~PIDController();
  virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = "", bool _is_yaw=false);

  double gain_p;
  double gain_i;
  double gain_d;
  double time_constant;
  double limit;

  double input=0.0;
  double dinput=0.0;
  double output=0.0;
  double p, i, d;
  bool is_yaw;

  double update(double input, double x, double dx, double dt);
  double wrapAngle(double angle);
  void reset();
};

#endif // PIDCONTROLLER_H
