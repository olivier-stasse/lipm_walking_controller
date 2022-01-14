#include <ExternalFootstepPlanner/InputConvertor.h>

namespace mc_plugin
{

namespace ExternalFootstepPlanner
{

SE2d InputConvertor::convert_PS4_to_SE2d(const sensor_msgs::Joy & joy_msg)
{
  SE2d result;
  result.x = joy_msg.axes.at(1);
  result.y = joy_msg.axes.at(0);
  result.theta = joy_msg.axes.at(1);
  return result;
}

SE2d InputConvertor::convert_Oculus_to_SE2d(const sensor_msgs::Joy & joy_msg)
{
  SE2d result;
  result.x = joy_msg.axes.at(1);
  result.y = joy_msg.axes.at(0);
  result.theta = joy_msg.axes.at(1);
  return result;
}

double InputConvertor::L2norm(double a_x, double a_y, double b_x, double b_y)
{
  return sqrt(pow(a_x - b_x, 2) + pow(a_y - b_y, 2));
}

double InputConvertor::Uniform_norm(double a_x, double a_y)
{
  if(abs(a_x) > abs(a_y))
  {
    return abs(a_x);
  }
  else
  {
    return abs(a_y);
  }
}

double InputConvertor::calc_exponential_interpolation(double exp, double y_max, double x)
{
  // calc y_max* x^exp  [0,1] -> [0,y_max]
  return y_max * pow(x, exp);
}

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin