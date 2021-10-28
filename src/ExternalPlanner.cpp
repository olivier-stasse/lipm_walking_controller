#include <mc_rtc/gui.h>

#include <lipm_walking/ExternalPlanner.h>

namespace lipm_walking
{

void ExternalPlanner::addGUIElements(mc_rtc::gui::StateBuilder & gui)
{
  using namespace mc_rtc::gui;
  gui.addElement({"Walking", "Footsteps", "ExternalPlanner"}, XYTheta("World target [m, rad]",
                                                                      [this]() -> std::array<double, 4> {
                                                                        const auto target = targetSE2d().vector();
                                                                        return {target.x(), target.y(), target.z(), 0.};
                                                                      },
                                                                      [this](const std::array<double, 4> & target) {
                                                                        targetSE2d({target[0], target[1], target[2]});
                                                                      }));
}

void ExternalPlanner::removeGUIElements(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"Walking", "Footsteps", "ExternalPlanner"});
}

const SE2d & ExternalPlanner::targetSE2d() const noexcept
{
  return target_;
}

void ExternalPlanner::targetSE2d(const SE2d & target)
{
  target_ = target;
}

} // namespace lipm_walking
