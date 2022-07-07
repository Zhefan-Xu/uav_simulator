#include <uav_simulator/quadcopterPlugin.h>

PIDController::PIDController()
{
    
}

PIDController::~PIDController(){
    
}


void PIDController::Load(sdf::ElementPtr _sdf, const std::string& prefix, bool _is_yaw)
{
  gain_p = 5.0;
  gain_d = 1.0;
  gain_i = 0.0;
  time_constant = 0.0;
  limit = -1.0;

  is_yaw = _is_yaw;

  if (!_sdf) return;
  if (_sdf->HasElement(prefix + "ProportionalGain")) gain_p = _sdf->GetElement(prefix + "ProportionalGain")->Get<double>();
  if (_sdf->HasElement(prefix + "DifferentialGain")) gain_d = _sdf->GetElement(prefix + "DifferentialGain")->Get<double>();
  if (_sdf->HasElement(prefix + "IntegralGain"))     gain_i = _sdf->GetElement(prefix + "IntegralGain")->Get<double>();
  if (_sdf->HasElement(prefix + "TimeConstant"))     time_constant = _sdf->GetElement(prefix + "TimeConstant")->Get<double>();
  if (_sdf->HasElement(prefix + "Limit"))            limit = _sdf->GetElement(prefix + "Limit")->Get<double>();

}

double PIDController::update(double new_input, double x, double dx, double dt)
{
  if (is_yaw){
    dx = wrapAngle(dx);
    new_input = wrapAngle(new_input);
    x = wrapAngle(x);
  }

  // limit command
  if (limit > 0.0 && fabs(new_input) > limit) new_input = (new_input < 0 ? -1.0 : 1.0) * limit;

  // filter command
  if (dt + time_constant > 0.0) {
    if (not is_yaw){
      input  = (dt * new_input + time_constant * input) / (dt + time_constant);
      dinput = (new_input - input) / (dt + time_constant);
    }
    else{
      input  = wrapAngle(dt * new_input + time_constant * input) / (dt + time_constant);
      dinput = wrapAngle(new_input - input) / (dt + time_constant);      
    }
  }

  // update proportional, differential and integral errors
  
  if (not is_yaw){
    p = input - x;
    d = dinput - dx;
    i = i + dt * p;
  }
  else{
    p = wrapAngle(input - x);
    d = wrapAngle(dinput - dx);
    i = wrapAngle(i + dt * p);
  }

  // update control output
  if (not is_yaw){
    output = gain_p * p + gain_d * d + gain_i * i;
  }
  else{
    output = gain_p * p + gain_d * d + gain_i * i;
  }
  return output;
}

double PIDController::wrapAngle(double angle){
  while (angle > M_PI){
    angle -= 2 * M_PI;
  }

  while (angle < -M_PI){
    angle += 2 * M_PI;
  }
  return angle;
}

void PIDController::reset()
{
  input = dinput = 0;
  p = i = d = output = 0;
}

