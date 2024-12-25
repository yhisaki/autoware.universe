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

#ifndef AUTOWARE__TRAJECTORY__UTILS__CLOSEST_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__CLOSEST_HPP_

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"

#include <Eigen/Core>

#include <functional>
#include <optional>
#include <vector>

namespace autoware::trajectory
{
namespace detail::impl
{
std::optional<double> closest_with_constraint_impl(
  const std::function<Eigen::Vector2d(const double & s)> & trajectory_compute,
  const std::vector<double> & bases, const Eigen::Vector2d & point,
  const std::function<bool(const double &)> & constraint);
}  // namespace detail::impl

template <class TrajectoryPointType, class ArgPointType>
std::optional<double> closest_with_constraint(
  const trajectory::Trajectory<TrajectoryPointType> & trajectory, const ArgPointType & point,
  const std::function<bool(const TrajectoryPointType &)> & constraint)
{
  using autoware::trajectory::detail::to_point;
  std::function<Eigen::Vector2d(const double & s)> trajectory_compute =
    [&trajectory](const double & s) {
      TrajectoryPointType point = trajectory.compute(s);
      Eigen::Vector2d result;
      result << to_point(point).x, to_point(point).y;
      return result;
    };
  Eigen::Vector2d p(to_point(point).x, to_point(point).y);
  return detail::impl::closest_with_constraint_impl(
    trajectory_compute, trajectory.get_internal_bases(), p, constraint);
}

template <class TrajectoryPointType, class ArgPointType>
double closest(
  const trajectory::Trajectory<TrajectoryPointType> & trajectory, const ArgPointType & point)
{
  return closest_with_constraint(
    trajectory, point, [](const TrajectoryPointType &) { return true; });
}
}  // namespace autoware::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__CLOSEST_HPP_
