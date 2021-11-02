#pragma once
#include <mc_control/MCController.h>

#include <ExternalFootstepPlanner/Plan.h>
#include <ExternalFootstepPlanner/Request.h>
#include <lipm_walking/FootstepPlan.h>
#include <lipm_walking/utils/SE2d.h>

namespace mc_control
{
struct MCController;
} // namespace mc_control

namespace lipm_walking
{

/**
 * @brief Handle requesting/receiving a footstep plan from an external planner
 *
 * For now this is intended to be used along with the mc_plugin_footstep_plan plugin that handles the actual
 * communication with the planner, and conversion to a FootstepPlan.
 */
struct ExternalPlanner
{
  using SE2d = lipm_walking::utils::SE2d;
  using DeferredPlan = mc_plugin::ExternalFootstepPlanner::DeferredPlan;
  using Request = mc_plugin::ExternalFootstepPlanner::Request;

  enum State
  {
    Standing,
    DoubleSupport,
    SingleSupport
  }; ///< State in which the requested plan applies

  ExternalPlanner(mc_control::MCController & ctl);
  void addGUIElements();
  void removeGUIElements();

  inline void targetSE2d(const SE2d & xytheta_z);
  inline const SE2d & targetSE2d() const noexcept;

  ///< Flag indicating that we need to request a plan
  bool planRequested() const
  {
    return requestPlan_;
  }

  void clearPlanRequested()
  {
    requestPlan_ = false;
  }

  void requestPlan(State state, const Request & request)
  {
    // static auto future = ctl_.datastore().call<DeferredPlan, const Request &>("ExternalFootstepPlanner::Request",
    // request);
    mc_rtc::log::info("ExternalPlanner requesting");
    futurePlan_ = ctl_.datastore().call<DeferredPlan, const Request &>("ExternalFootstepPlanner::Request", request);
    state_ = state;
  }

  DeferredPlan & plan()
  {
    return futurePlan_;
  }

  State state() const
  {
    return state_;
  }

protected:
  mc_control::MCController & ctl_;
  DeferredPlan futurePlan_; ///< This object will contain the footstep plan once computed
  State state_ = State::Standing;
  SE2d target_; // Target expressed as X,Y,theta
  bool requestPlan_ = false;
};

} // namespace lipm_walking
