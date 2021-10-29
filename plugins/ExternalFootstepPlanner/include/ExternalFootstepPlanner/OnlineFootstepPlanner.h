#pragma once

#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

/**
 * @brief Base interface to interact with external planners
 */
struct OnlineFootstepPlanner : ExternalFootstepPlanner
{
  /**
   * @brief
   *
   * @param request Requested parameters for the plan (start, finish, time, etc)
   * @return std::future<boost::optional<Plan>> A future plan to be returned once processed
   */
  std::future<boost::optional<Plan>> requestPlan(const Request & request) override;
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin