#include <ExternalFootstepPlanner/OnlineFootstepPlanner.h>
#include <chrono>
#include <thread>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{
std::future<boost::optional<Plan>> OnlineFootstepPlanner::requestPlan(const Request & /*request*/)
{
  return std::async([]() {
    boost::optional<Plan> plan;
    // Dummy wait to check
    std::this_thread::sleep_for(std::chrono::seconds(3));
    // If we have a plan, assign it, otherwise this means the planner did't return a plan
    // plan = Plan{};
    return plan;
  });
}
} // namespace ExternalFootstepPlanner

} // namespace mc_plugin
