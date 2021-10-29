#pragma once

#include <boost/optional.hpp>

#include <ExternalFootstepPlanner/Plan.h>
#include <ExternalFootstepPlanner/Request.h>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

/**
 * @brief Base interface to interact with external planners
 */
struct ExternalFootstepPlanner
{
  /**
   * @brief
   *
   * @param request Requested parameters for the plan (start, finish, time, etc)
   * @return std::future<boost::optional<Plan>> A future plan to be returned once processed
   */
  virtual DeferredPlan requestPlan(const Request & request) = 0;
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin