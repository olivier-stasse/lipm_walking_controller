#pragma once

#include <Eigen/Core>
#include <vector>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{
struct Plan
{
  inline const std::vector<Eigen::Vector3d> & contacts() const noexcept
  {
    return contacts_;
  }

protected:
  std::vector<Eigen::Vector3d> contacts_;
};
} // namespace ExternalFootstepPlanner
} // namespace mc_plugin