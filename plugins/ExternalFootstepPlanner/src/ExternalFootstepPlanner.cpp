#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/ros.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>

namespace mc_plugin
{

namespace ExternalFootstepPlanner
{

void ExternalFootstepPlanner::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & /* config */)
{
  mc_rtc::log::success("[ExternalFootstepPlanner] Plugin initialized");
  reset(gc);
}

void ExternalFootstepPlanner::reset(mc_control::MCGlobalController & /* gc */)
{
  mc_rtc::log::success("[ExternalFootstepPlanner] Plugin reset");
}

void ExternalFootstepPlanner::before(mc_control::MCGlobalController &) {}

mc_control::GlobalPlugin::GlobalPluginConfiguration ExternalFootstepPlanner::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = false;
  return out;
}

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ExternalFootstepPlanner", mc_plugin::ExternalFootstepPlanner::ExternalFootstepPlanner)