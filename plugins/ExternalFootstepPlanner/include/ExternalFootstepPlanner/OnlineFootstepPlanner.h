#pragma once

#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>
#include <ExternalFootstepPlanner/Plan.h>
#include <future>
#include <mutex>
#include <thread>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

/**
 * @brief Base interface to interact with external planners
 */
struct OnlineFootstepPlanner : ExternalFootstepPlanner
{
  OnlineFootstepPlanner();
  ~OnlineFootstepPlanner();

  /**
   * @brief
   *
   * @param request Requested parameters for the plan (start, finish, time, etc)
   * @return std::future<boost::optional<Plan>> A future plan to be returned once processed
   */
  void requestPlan(const Request & request) override;
  bool hasPlan() const noexcept override;
  Plan popPlan() override;

  std::string name() const override
  {
    return "OnlineFootstepPlanner";
  }

protected:
  void rosThread();

protected:
  std::thread rosThread_;
  std::atomic<bool> planRequested_{false};
  std::atomic<bool> planReceived_{false};
  std::atomic<bool> run_{true};
  double rate_ = 30;
  mutable std::mutex requestMutex_;
  Request request_;
  mutable std::mutex planMutex_;
  Plan plan_;

  std::future<boost::optional<Plan>> futurePlan_;
  std::string footstep_service_topic_ =
      "/online_footstep_planner/footstep_generation_srv"; ///< ROS service used to request a new plan
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin