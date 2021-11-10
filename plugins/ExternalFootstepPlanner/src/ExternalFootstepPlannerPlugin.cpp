#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/io_utils.h>
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
  config_ = config;
  config("category", category_);

  auto plannerName = config("planner", std::string{"OnlineFootstepPlanner"});
  changePlanner(plannerName);

  ctl.datastore().make_call("ExternalFootstepPlanner::Available", [this]() { return planner_->available(); });
  ctl.datastore().make_call("ExternalFootstepPlanner::Activate", [this, &gui]() { activate(gui); });
  ctl.datastore().make_call("ExternalFootstepPlanner::Deactivate", [this, &gui]() { deactivate(gui); });
  // Do we need replanning?
  ctl.datastore().make_call("ExternalFootstepPlanner::PlanningRequested",
                            [this]() { return worldPositionTargetChanged_ || localPositionTargetChanged_; });
  ctl.datastore().make_call("ExternalFootstepPlanner::WorldPositionTargetChanged",
                            [this]() { return worldPositionTargetChanged_; });
  ctl.datastore().make_call("ExternalFootstepPlanner::WorldPositionTarget",
                            [this]() -> const SE2d & { return worldPositionTarget_; });
  ctl.datastore().make_call("ExternalFootstepPlanner::LocalPositionTargetChanged",
                            [this]() { return localPositionTargetChanged_; });
  ctl.datastore().make_call("ExternalFootstepPlanner::LocalPositionTarget",
                            [this]() -> const SE2d & { return localPositionTarget_; });
  ctl.datastore().make_call("ExternalFootstepPlanner::SetWorldPositionTarget", [this](const SE2d & worldTarget) {
    worldPositionTargetChanged_ = true;
    worldPositionTarget_ = worldTarget;
  });
  ctl.datastore().make_call("ExternalFootstepPlanner::SetLocalPositionTarget", [this](const SE2d & localTarget) {
    localPositionTargetChanged_ = true;
    localPositionTarget_ = localTarget;
  });
  // Call this to request a new plan
  ctl.datastore().make_call("ExternalFootstepPlanner::RequestPlan", [this](const Request & request) {
    worldPositionTargetChanged_ = false;
    localPositionTargetChanged_ = false;
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
  auto & gui = *ctl.gui();
  if(wasAvailable_ && !planner_->available())
  {
    removePlannerGUI(gui);
    wasAvailable_ = false;
  }
  else if(!wasAvailable_ && planner_->available())
  {
    addPlannerGUI(gui);
    wasAvailable_ = true;
  }
}

mc_control::GlobalPlugin::GlobalPluginConfiguration ExternalFootstepPlannerPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = false;
  return out;
}

void ExternalFootstepPlannerPlugin::changePlanner(const std::string & plannerName)
{
  if(plannerName == plannerName_ && planner_)
  {
    return;
  }

  if(plannerName == "OnlineFootstepPlanner")
  {
    planner_.reset(new OnlineFootstepPlanner{});
    if(config_.has("OnlineFootstepPlanner"))
    {
      planner_->configure(config_("OnlineFootstepPlanner"));
    }
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[{}] does not support planner {} (supported planners are: {})",
                                                        name(), plannerName, mc_rtc::io::to_string(supportedPlanners_));
  }
  mc_rtc::log::success("[{}] Changed planner to {}", name(), plannerName);
}

void ExternalFootstepPlannerPlugin::activate(mc_rtc::gui::StateBuilder & gui)
{
  if(activated_) return;

  using namespace mc_rtc::gui;
  gui.addElement(category_, ComboInput("Planner", supportedPlanners_, [this]() { return plannerName_; },
                                       [this](const std::string & planner) { changePlanner(planner); }));
  gui.addElement(category_, Label("Available?", [this]() { return planner_->available(); }));
  planner_->activate();
  activated_ = true;
}

void ExternalFootstepPlannerPlugin::deactivate(mc_rtc::gui::StateBuilder & gui)
{
  if(!activated_) return;
  using namespace mc_rtc::gui;
  gui.removeElement(category_, "Planner");
  gui.removeElement(category_, "Available?");
  removePlannerGUI(gui);
  planner_->deactivate();
  activated_ = false;
  wasAvailable_ = false;
}

void ExternalFootstepPlannerPlugin::addPlannerGUI(mc_rtc::gui::StateBuilder & gui)
{
  using namespace mc_rtc::gui;
  gui.addElement(category_,
                 XYTheta("World target [m, rad]",
                         [this]() -> std::array<double, 4> {
                           return {worldPositionTarget_.x, worldPositionTarget_.y, worldPositionTarget_.theta, 0.};
                         },
                         [this](const std::array<double, 4> & target) {
                           worldPositionTarget_.x = target[0];
                           worldPositionTarget_.y = target[1];
                           worldPositionTarget_.theta = target[2];
                           worldPositionTargetChanged_ = true;
                         }),
                 ArrayInput("Local target [m, rad]",
                            [this]() -> std::array<double, 3> {
                              return {localPositionTarget_.x, localPositionTarget_.y, localPositionTarget_.theta};
                            },
                            [this](const std::array<double, 3> & target) {
                              localPositionTarget_.x = target[0];
                              localPositionTarget_.y = target[1];
                              localPositionTarget_.theta = target[2];
                              localPositionTargetChanged_ = true;
                            }));
}

void ExternalFootstepPlannerPlugin::removePlannerGUI(mc_rtc::gui::StateBuilder & gui)
{
  using namespace mc_rtc::gui;
  gui.removeElement(category_, "World target [m, rad]");
}

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ExternalFootstepPlannerPlugin", mc_plugin::ExternalFootstepPlanner::ExternalFootstepPlannerPlugin)