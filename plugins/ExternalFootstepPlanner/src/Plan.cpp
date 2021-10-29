#include <ExternalFootstepPlanner/Plan.h>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

bool DeferredPlan::ready() const noexcept
{
  return internal::is_ready(futurePlan_);
}

boost::optional<Plan> DeferredPlan::get()
{
  if(ready())
  {
    return futurePlan_.get();
  }
  return {};
}
} // namespace ExternalFootstepPlanner
} // namespace mc_plugin