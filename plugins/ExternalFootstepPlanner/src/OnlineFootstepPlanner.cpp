#include <mc_rtc/logging.h>

#include <ExternalFootstepPlanner/OnlineFootstepPlanner.h>
#include <chrono>
#include <thread>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{
DeferredPlan OnlineFootstepPlanner::requestPlan(const Request & /*request*/)
{
  static int nreq = 0;
  nreq++;
  return {[nreq]() {
    mc_rtc::log::info("Starting plan thread number {}", nreq);
    boost::optional<Plan> plan;
    // Dummy wait to check
    std::this_thread::sleep_for(std::chrono::seconds(3));
    // If we have a plan, assign it, otherwise this means the planner did't return a plan
    // plan = Plan{};
    mc_rtc::log::info("Stopping plan thread number {}", nreq);
    return plan;
  }};
}
} // namespace ExternalFootstepPlanner

} // namespace mc_plugin
