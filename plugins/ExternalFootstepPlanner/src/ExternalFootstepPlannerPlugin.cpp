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
  auto & ctl = gc.controller();

  auto name = config("planner", std::string{"OnlineFootstepPlanner"});
  if(name == "OnlineFootstepPlanner")
  {
    planner_.reset(new OnlineFootstepPlanner{});
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[ExternalFootstepPlanner] does not support planner {}", name);
  }

  ctl.datastore().make_call("ExternalFootstepPlanner::Request",
                            [this](const Request & request) { return planner_->requestPlan(request); });

  mc_rtc::log::success("[ExternalFootstepPlanner] Plugin initialized");
  reset(gc);
}

void ExternalFootstepPlannerPlugin::reset(mc_control::MCGlobalController &)
{
  mc_rtc::log::success("[ExternalFootstepPlanner] Plugin reset");
}

void ExternalFootstepPlannerPlugin::before(mc_control::MCGlobalController & gc)
{
  auto & ctl = gc.controller();

  // // try dummy request
  static auto futurePlan = ctl.datastore().call<DeferredPlan, const Request &>("ExternalFootstepPlanner::Request",
                                                                               static_cast<const Request &>(Request{}));

  // if(futurePlan.ready())
  //{
  //  mc_rtc::log::success("Received dummy plan!");
  //  auto result = futurePlan.get();
  //  if(result)
  //  {
  //    mc_rtc::log::info("Planner returned a plan");
  //  }
  //  else
  //  {
  //    mc_rtc::log::warning("Planner did not return a plan");
  //  }
  //}
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

EXPORT_MC_RTC_PLUGIN("ExternalFootstepPlanner", mc_plugin::ExternalFootstepPlanner::ExternalFootstepPlannerPlugin)