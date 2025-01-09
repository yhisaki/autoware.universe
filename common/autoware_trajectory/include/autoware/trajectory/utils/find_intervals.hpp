// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__TRAJECTORY__UTILS__FIND_INTERVALS_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__FIND_INTERVALS_HPP_

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"

#include <Eigen/Core>

#include <functional>
#include <vector>
namespace autoware::trajectory
{
struct Interval
{
  double start;
  double end;
};

namespace detail::impl
{
std::vector<Interval> find_intervals_impl(
  const std::vector<double> & bases, const std::function<bool(const double &)> & constraint);
};  // namespace detail::impl

template <class TrajectoryPointType, class Constraint>
std::vector<Interval> find_intervals(
  const Trajectory<TrajectoryPointType> & trajectory, const Constraint & constraint)
{
  using autoware::trajectory::detail::to_point;

  return detail::impl::find_intervals_impl(
    trajectory.get_internal_bases(),
    [&constraint, &trajectory](const double & s) { return constraint(trajectory.compute(s)); });
}

}  // namespace autoware::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__FIND_INTERVALS_HPP_