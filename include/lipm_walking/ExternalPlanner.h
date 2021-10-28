#pragma once
#include <lipm_walking/utils/SE2d.h>

namespace mc_rtc
{
namespace gui
{
struct StateBuilder;
} // namespace gui
} // namespace mc_rtc

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
  void addGUIElements(mc_rtc::gui::StateBuilder & gui);
  void removeGUIElements(mc_rtc::gui::StateBuilder & gui);

  inline void targetSE2d(const SE2d & xytheta_z);
  inline const SE2d & targetSE2d() const noexcept;
  // void requestPlan();
  // void receivePlan();
  // bool hasNewPlan();
protected:
  SE2d target_; // Target expressed as X,Y,theta
};

} // namespace lipm_walking
