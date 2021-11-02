#pragma once

#include <boost/optional.hpp>

#include <Eigen/Core>
#include <future>
#include <vector>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

namespace internal
{
/**
 * @brief Checks whether a future is ready (computation finished)
 *
 * @tparam R Type of future to check
 * @param f Future to be checked
 * @return true If the future has finished
 */
template<typename R>
bool is_ready(std::future<R> const & f)
{
  return f.valid() && f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

} // namespace internal

struct Plan
{
  inline const std::vector<Eigen::Vector3d> & contacts() const noexcept
  {
    return contacts_;
  }

protected:
  std::vector<Eigen::Vector3d> contacts_;
};

/**
 * @brief Wrapper around an asynchronous plan computation
 *
 * Expects a functor that will run asynchronously to compute the plan
 * This computation can take time, thus is is run asynchronously.
 *
 * You can check whether the computation has finished using ready(), upon which the plan
 * will be available using get().
 */
struct DeferredPlan
{
  // Move only
  DeferredPlan(const DeferredPlan &) = delete;
  DeferredPlan & operator=(const DeferredPlan &) = delete;
  DeferredPlan(DeferredPlan &&) = default;
  DeferredPlan & operator=(DeferredPlan &&) = default;

  DeferredPlan() {}

  template<typename Func>
  DeferredPlan(Func f)
  {
    futurePlan_ = std::async(f);
  }

  /**
   * @brief Checks whether the plan computation has finished
   *
   * @return true when the plan is available
   * @return false otherwise
   */

  bool ready() const noexcept;
  /**
   * @brief This function will only return a valid plan result if ready() is true.
   * The plan will be returned only on the first call to this function. All subsequent calls will
   * return an empty optional
   *
   * @return boost::optional<Plan> The computed plan if and only if ready() is true. This function is intended to be
   * called only once.
   */
  boost::optional<Plan> get();

protected:
  std::future<boost::optional<Plan>> futurePlan_;
  boost::optional<Plan> plan_; ///< Only has value when the future (async task) has completed
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin