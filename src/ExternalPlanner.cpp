#include <mc_control/MCController.h>
#include <mc_rtc/gui.h>

#include <boost/optional.hpp>

#include <ExternalFootstepPlanner/Plan.h>
#include <lipm_walking/ExternalPlanner.h>

namespace lipm_walking
{

using SE2d = utils::SE2d;

ExternalPlanner::ExternalPlanner(mc_control::MCController & ctl) : ctl_(ctl) {}

void ExternalPlanner::requestPlan(const State state,
                                  const Foot supportFoot,
                                  const utils::SE2d & start_lf,
                                  const utils::SE2d & start_rf,
                                  double allowed_time)
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
  request.allowed_time = allowed_time;

  ctl_.datastore().call<void>("ExternalFootstepPlanner::RequestPlan", static_cast<const Request &>(request));
  state_ = state;
  requested_ = true;
}

std::vector<lipm_walking::Contact> ExternalPlanner::plan()
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

  requested_ = false;
  auto plan = ctl_.datastore().call<mc_plugin::ExternalFootstepPlanner::Plan>("ExternalFootstepPlanner::PopPlan");
  return convertPlan(plan);
}

} // namespace lipm_walking
