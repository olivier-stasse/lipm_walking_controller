#pragma once

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

struct SE2d
{
  double x = 0;
  double y = 0;
  double theta = 0;
};

/**
 * @brief For now the request is simply the initial position of the feet, and goal position, along with a maximum
 * allowed computation time
 */
struct Request
{
  SE2d start_left_foot, start_right_foot; ///< Starting state (x;y;theta) for the feet. Should match the state in which
                                          ///< the robot is expected to be once the plan will be applied
  SE2d goal_left_foot, goal_right_foot; ///< Goal state for the feet. This is where you want each foot to be at the end
                                        ///< of the footstep plan.
  double max_time = 0.5;
};

} // namespace ExternalFootstepPlanner

} // namespace mc_plugin
