#include <mc_control/MCController.h>
#include <mc_rtc/gui.h>

#include <boost/optional.hpp>

#include <ExternalFootstepPlanner/Plan.h>
#include <lipm_walking/ExternalPlanner.h>

namespace lipm_walking
{

using SE2d = utils::SE2d;

ExternalPlanner::ExternalPlanner(mc_control::MCController & ctl) : ctl_(ctl) {}

} // namespace lipm_walking
