/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>
#include <ExternalFootstepPlanner/SE2d.h>

namespace mc_plugin
{

namespace ExternalFootstepPlanner
{

/**
 * @brief This plugin handles communication with external footstep planners and constructs a footstep plan that can be
 * executed by walking controllers
 *
 * Currently supports:
 * - OnlineFootstepPlanner: https://github.com/isri-aist/OnlineFootstepPlanner
 */
struct ExternalFootstepPlannerPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController &) override {}

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  inline const std::string & name() const noexcept
  {
    return name_;
  }

protected:
  std::unique_ptr<ExternalFootstepPlanner> planner_;

  std::string name_{"ExternalFootstepPlanner"};
  std::vector<std::string> category_{"ExternalFootStepPlanner"};
  bool targetChanged_ = false;
  SE2d target_;
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin