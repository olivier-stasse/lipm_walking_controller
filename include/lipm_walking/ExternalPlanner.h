#pragma once
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

  ExternalPlanner(mc_control::MCController & ctl);
  void addGUIElements();
  void removeGUIElements();

  inline void targetSE2d(const SE2d & xytheta_z);
  inline const SE2d & targetSE2d() const noexcept;

  // True once the last plan requested through requestPlan has been received, false otherwise
  inline bool hasPlan() const noexcept
  {
    return hasPlan_;
  }

  const lipm_walking::FootstepPlan & pop_plan() noexcept
  {
    hasPlan_ = false;
    return plan_;
  }

  inline const lipm_walking::FootstepPlan & plan() const noexcept
  {
    return plan_;
  }

protected:
  void requestPlan();

protected:
  mc_control::MCController & ctl_;
  SE2d target_; // Target expressed as X,Y,theta

  lipm_walking::FootstepPlan plan_;
  bool hasPlan_ = false;
};

} // namespace lipm_walking
