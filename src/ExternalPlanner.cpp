#include <mc_control/MCController.h>
#include <mc_rtc/gui.h>

#include <boost/optional.hpp>

#include <ExternalFootstepPlanner/Plan.h>
#include <lipm_walking/ExternalPlanner.h>

namespace lipm_walking
{

using SE2d = utils::SE2d;

ExternalPlanner::ExternalPlanner(mc_control::MCController & ctl) : ctl_(ctl)
{

  ctl_.datastore().make_call("Plugins::FSP::WalkTo", [this](double x, double y, double theta) {
    targetSE2d({x, y, theta});
  });

  // Create a callback that will be called by the plugin to receive the new plan
  ctl_.datastore().remove("Plugins::FSP::PlanCallback");
  ctl_.datastore().make_call("Plugins::FSP::PlanCallback",
                             [this](const boost::optional<mc_plugin::ExternalFootstepPlanner::Plan> & plan_) {
                               if(!plan_)
                               {
                                 mc_rtc::log::error("[ExternalPlanner] Last plan request failed");
                                 return;
                               }
                               const auto & plan = *plan_;
                               mc_rtc::log::info("Received new plan with {} footsteps", plan.contacts().size());
                               // plan_ = plan;
                               hasPlan_ = true;
                             });
}

void ExternalPlanner::addGUIElements()
{
  using namespace mc_rtc::gui;
  auto & gui = *ctl_.gui();
  // XXX should we use the GUI from the plugin here instead?
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
  // auto & gui = *ctl_.gui();
  // gui.removeCategory({"Walking", "Footsteps", "ExternalPlanner"});
}

const SE2d & ExternalPlanner::targetSE2d() const noexcept
{
  return target_;
}

void ExternalPlanner::targetSE2d(const SE2d & target)
{
  target_ = target;
  // Target changed, request a new plan
  requestPlan();
}

void ExternalPlanner::requestPlan()
{
  if(!ctl_.datastore().has("Plugins::FSP"))
  {
    mc_rtc::log::critical("Footstep Planning plugin is unavailable, external plans will not be generated");
    return;
  }

  hasPlan_ = false;

  // Call mc_plugin_footstep plan to request a new plan for the given target
  // Use the service to retrieve the plan
  // ctl.datastore().call("Plugins::FSP::RequestPlan(target)")
  // The plugin should be calling the OnlineFootstepPlanner service and waiting for the answer in a thread
  // Once the plugin answer is received, it should make the plan available to the planner
  // If we received it in time (before the end of Standing state), then we can use it, else we drop the plan
  // Should be calling FSP::updateWorldTarget(target)
  if(ctl_.datastore().has("Plugins::FSP::RequestPlan"))
  {
    // if the planner is already computing and we send a new request, it should stop computing and start replanning
    // straight away
    // FIXME (datastore): the datastore should allow to call a const T & with a T & and do the cast by itself
    ctl_.datastore().call("Plugins::FSP::RequestPlan", static_cast<const SE2d &>(target_));
  }
}

} // namespace lipm_walking
