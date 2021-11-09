#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/ros.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <ExternalFootstepPlanner/ExternalFootstepPlannerPlugin.h>
#include <ExternalFootstepPlanner/OnlineFootstepPlanner.h>

namespace mc_plugin
{

namespace ExternalFootstepPlanner
{

void ExternalFootstepPlannerPlugin::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  using namespace mc_rtc::gui;
  auto & ctl = gc.controller();
  auto & gui = *ctl.gui();

  auto plannerName = config("planner", std::string{"OnlineFootstepPlanner"});
  if(plannerName == "OnlineFootstepPlanner")
  {
    planner_.reset(new OnlineFootstepPlanner{});
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[{}] does not support planner {}", name(), plannerName);
  }

  gui.addElement(category_, Label("Planner", [this]() { return planner_->name(); }),
                 XYTheta("World target [m, rad]",
                         [this]() -> std::array<double, 4> {
                           return {target_.x, target_.y, target_.theta, 0.};
                         },
                         [this](const std::array<double, 4> & target) {
                           target_.x = target[0];
                           target_.y = target[1];
                           target_.theta = target[2];
                           targetChanged_ = true;
                         }));

  // Do we need replanning?
  ctl.datastore().make_call("ExternalFootstepPlanner::PlanningRequested", [this]() { return targetChanged_; });
  ctl.datastore().make_call("ExternalFootstepPlanner::Target", [this]() { return target_; });
  // Call this to request a new plan
  ctl.datastore().make_call("ExternalFootstepPlanner::RequestPlan", [this](const Request & request) {
    targetChanged_ = false;
    planner_->requestPlan(request);
  });
  ctl.datastore().make_call("ExternalFootstepPlanner::HasPlan", [this]() { return planner_->hasPlan(); });
  ctl.datastore().make_call("ExternalFootstepPlanner::PopPlan", [this]() { return planner_->popPlan(); });

  mc_rtc::log::success("[{}] Plugin initialized", name());
  reset(gc);
}

void ExternalFootstepPlannerPlugin::reset(mc_control::MCGlobalController &)
{
  mc_rtc::log::success("[{}] Plugin reset", name());
}

void ExternalFootstepPlannerPlugin::before(mc_control::MCGlobalController & gc)
{
  auto & ctl = gc.controller();
}

mc_control::GlobalPlugin::GlobalPluginConfiguration ExternalFootstepPlannerPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = false;
  return out;
}

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ExternalFootstepPlannerPlugin", mc_plugin::ExternalFootstepPlanner::ExternalFootstepPlannerPlugin)