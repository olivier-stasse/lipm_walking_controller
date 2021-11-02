#include <mc_control/MCController.h>
#include <mc_rtc/gui.h>

#include <boost/optional.hpp>

#include <ExternalFootstepPlanner/Plan.h>
#include <lipm_walking/ExternalPlanner.h>

namespace lipm_walking
{

using SE2d = utils::SE2d;

ExternalPlanner::ExternalPlanner(mc_control::MCController & ctl) : ctl_(ctl) {}

void ExternalPlanner::addGUIElements()
{
  using namespace mc_rtc::gui;
  auto & gui = *ctl_.gui();
  gui.addElement({"Walking", "Footsteps", "ExternalPlanner"}, XYTheta("World target [m, rad]",
                                                                      [this]() -> std::array<double, 4> {
                                                                        const auto target = targetSE2d().vector();
                                                                        return {target.x(), target.y(), target.z(), 0.};
                                                                      },
                                                                      [this](const std::array<double, 4> & target) {
                                                                        targetSE2d({target[0], target[1], target[2]});
                                                                      }));
}

void ExternalPlanner::removeGUIElements()
{
  auto & gui = *ctl_.gui();
  gui.removeCategory({"Walking", "Footsteps", "ExternalPlanner"});
}

const SE2d & ExternalPlanner::targetSE2d() const noexcept
{
  return target_;
}

void ExternalPlanner::targetSE2d(const SE2d & target)
{
  target_ = target;
  // Target changed, request a new plan
  requestPlan_ = true;
}

} // namespace lipm_walking
