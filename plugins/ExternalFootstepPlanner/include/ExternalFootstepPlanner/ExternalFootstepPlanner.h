#pragma once

#include <mc_rtc/gui/StateBuilder.h>

#include <ExternalFootstepPlanner/Plan.h>
#include <ExternalFootstepPlanner/Request.h>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

/**
 * @brief Base interface to interact with external planners
 */
struct ExternalFootstepPlanner
{
  virtual void configure(const mc_rtc::Configuration &){};
  /**
   * @brief
   *
   * @param request Requested parameters for the plan (start, finish, time, etc)
   * @return std::future<boost::optional<Plan>> A future plan to be returned once processed
   */
  virtual void requestPlan(const Request & request) = 0;
  virtual bool hasPlan() const noexcept = 0;
  virtual Plan popPlan() = 0;
  virtual std::string name() const = 0;
  virtual bool available() const = 0;
  virtual void addToGUI(mc_rtc::gui::StateBuilder &){};
  virtual void removeFromGUI(mc_rtc::gui::StateBuilder &){};
  virtual void activate() = 0;
  virtual void deactivate() = 0;
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin