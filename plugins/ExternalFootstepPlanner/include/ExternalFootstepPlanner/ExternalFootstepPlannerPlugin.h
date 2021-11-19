/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>
#include <ExternalFootstepPlanner/ExternalFootstepPlannerPlugin.h>
#include <ExternalFootstepPlanner/SE2d.h>

namespace mc_plugin
{

namespace ExternalFootstepPlanner
{

/**
 * @brief This plugin handles external footstep planning

 * It currently supports position-based planners, that expect as input:
 * - The initial support feet state (x, y, theta)
 * - The final support feet state (x, y, theta)
 *
 * Communication between the plugin and your controller is done through the datastore.
 *
 * Currently supports:
 * - OnlineFootstepPlanner: https://github.com/isri-aist/OnlineFootstepPlanner
 *   This planner is a modified version of the humanoid_navigation ROS stack.
 *   It supports planning with obstacles avoidance, online replanning and more, and can be
 *   controlled through ROS services.
 *   See also:
 *   - The OnlineFootstepPlanner project
 *   - @see OnlineFootstepPlanner
 */
struct ExternalFootstepPlannerPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;
  void reset(mc_control::MCGlobalController & controller) override;
  void before(mc_control::MCGlobalController &) override;
  void after(mc_control::MCGlobalController &) override {}
  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  /**
   * @brief Name of this plugin
   *
   * @return const std::string&
   */
  inline const std::string & name() const noexcept
  {
    return name_;
  }

protected:
  void changePlanner(const std::string & plannerName);
  void changeTargetType(const std::string & targetType);
  /**
   * @brief Add GUI elements that are visible when the planner is available
   */
  void addPlannerGUI();
  /**
   * @brief Remove elements added by addPlannerGUI()
   */
  void removePlannerGUI();
  /**
   * @brief Activates the plugin (add it to the GUI) and the planner
   */
  void activate();
  /**
   * @brief Deactivate the plugin and the planner
   *
   * Can be reactivated by activate()
   */
  void deactivate();

protected:
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_{nullptr};
  bool activated_ = false; ///< Whether this plugin is active
  mc_rtc::Configuration config_; ///< Stores the plugin configuration, in particular the planners' options
  std::string name_{"ExternalFootstepPlanner"}; ///< Name of the plugin (mainly used for logging)
  std::vector<std::string> category_{"ExternalFootStepPlanner"}; ///< Category in the gui

  std::vector<std::string> supportedPlanners_{"OnlineFootstepPlanner"}; ///< List of supported planners
  std::string plannerName_{"OnlineFootstepPlanner"}; ///< Name of the currently activated planner
  std::unique_ptr<ExternalFootstepPlanner> planner_{nullptr}; ///< Planner implementation
  bool wasAvailable_ = false; ///< True if the planner was active during the previous iteration

  std::vector<std::string> supportedTargetTypes_{"World SE2", "Local SE2", "Local Velocity"};
  std::string targetType_{"World SE2"};

  bool worldPositionTargetChanged_ =
      false; ///< True if the requested target has changed (either through the GUI or other means)
  SE2d worldPositionTarget_{}; ///< Requested target

  bool localPositionTargetChanged_{false};
  SE2d localPositionTarget_{};
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin