#pragma once
#include <mc_control/MCController.h>

#include <ExternalFootstepPlanner/Plan.h>
#include <ExternalFootstepPlanner/Request.h>
#include <lipm_walking/FootstepPlan.h>
#include <lipm_walking/utils/SE2d.h>

namespace mc_control
{
struct MCController;
} // namespace mc_control

namespace lipm_walking
{

/**
 * @brief Handle requesting/receiving a footstep plan from an external planner
 *
 * For now this is intended to be used along with the mc_plugin_footstep_plan plugin that handles the actual
 * communication with the planner, and conversion to a FootstepPlan.
 */
struct ExternalPlanner
{
  using SE2d = lipm_walking::utils::SE2d;
  using Request = mc_plugin::ExternalFootstepPlanner::Request;
  using Foot = mc_plugin::ExternalFootstepPlanner::Foot;

  enum State
  {
    Standing,
    DoubleSupport,
    SingleSupport
  }; ///< State in which the requested plan applies

  ExternalPlanner(mc_control::MCController & ctl);

  bool planningRequested() const
  {
    return ctl_.datastore().call<bool>("ExternalFootstepPlanner::PlanningRequested");
  }

  void requestPlan(const State state,
                   const Foot supportFoot,
                   const utils::SE2d & start_lf,
                   const utils::SE2d & start_rf)
  {
    Request request;
    request.start_left_foot = {start_lf.x, start_lf.y, start_lf.theta};
    request.start_right_foot = {start_rf.x, start_rf.y, start_rf.theta};

    // Compute goal with nominal foot standing width
    auto target = ctl_.datastore().call<mc_plugin::ExternalFootstepPlanner::SE2d>("ExternalFootstepPlanner::Target");
    auto goal = utils::SE2d{target.x, target.y, target.theta};
    double width = 0.2; // XXX hardcoded
    auto lfOffset = utils::SE2d{0., width / 2, 0.0};
    auto rfOffset = utils::SE2d{0., -width / 2, 0.0};
    auto lfGoal = utils::SE2d(lfOffset.asPTransform() * goal.asPTransform());
    auto rfGoal = utils::SE2d(rfOffset.asPTransform() * goal.asPTransform());

    request.goal_left_foot = {lfGoal.x, lfGoal.y, lfGoal.theta};
    request.goal_right_foot = {rfGoal.x, rfGoal.y, rfGoal.theta};
    request.support_foot = supportFoot;

    ctl_.datastore().call<void>("ExternalFootstepPlanner::RequestPlan", static_cast<const Request &>(request));
    state_ = state;
  }

  bool hasPlan(State state)
  {
    return state_ == state && ctl_.datastore().call<bool>("ExternalFootstepPlanner::HasPlan");
  }

  /**
   * @brief Convert plugin's plan to lipm_walking plan format
   *
   * @return lipm_walking::FootstepPlan The plan to be executed
   */
  std::vector<lipm_walking::Contact> plan() const
  {
    auto convertPlan = [](const mc_plugin::ExternalFootstepPlanner::Plan & ext_plan) {
      std::vector<lipm_walking::Contact> contacts;
      unsigned i = 0;
      for(const auto & ext_contact : ext_plan.contacts)
      {
        auto contact = lipm_walking::Contact{};
        // contact.
        // plan.contacts.push_back()
        contact.surfaceName = (ext_contact.foot == Foot::Right ? "RightFootCenter" : "LeftFootCenter");
        utils::SE2d pose2D = {ext_contact.pose.x, ext_contact.pose.y, ext_contact.pose.theta};
        contact.pose = pose2D.asPTransform();
        contact.id = i++;
        contacts.push_back(contact);
      }
      return contacts;
    };

    auto plan = ctl_.datastore().call<mc_plugin::ExternalFootstepPlanner::Plan>("ExternalFootstepPlanner::PopPlan");
    return convertPlan(plan);
  }

  State state() const
  {
    return state_;
  }

protected:
  mc_control::MCController & ctl_;
  State state_ = State::Standing;
};

} // namespace lipm_walking
