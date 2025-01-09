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

#ifndef AUTOWARE__TRAJECTORY__UTILS__SHIFT_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__SHIFT_HPP_

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"

#include <optional>
#include <vector>

namespace autoware::trajectory
{

struct ShiftInterval
{
  double start{0.0};
  double end{0.0};
  double length{0.0};
};

struct ShiftParameters
{
  double velocity{0.0};
  double longitudinal_acc{0.0};
  double lateral_acc_limit{-1.0};
};

namespace detail::impl
{

void shift_impl(
  const std::vector<double> & bases, std::vector<double> * shift_lengths,
  const ShiftInterval & shift_interval, const ShiftParameters & shift_parameters);

}  // namespace detail::impl

template <class PointType>
trajectory::Trajectory<PointType> shift(
  const trajectory::Trajectory<PointType> & reference_trajectory,
  const std::vector<ShiftInterval> & shift_intervals, const ShiftParameters & shift_parameters = {})
{
  auto bases = reference_trajectory.get_internal_bases();
  std::vector<double> shift_lengths(bases.size(), 0.0);
  for (const auto & shift_interval : shift_intervals) {
    detail::impl::shift_impl(bases, &shift_lengths, shift_interval, shift_parameters);
  }
  // Apply shift.
  std::vector<PointType> shifted_points;
  for (size_t i = 0; i < bases.size(); ++i) {
    shifted_points.emplace_back(reference_trajectory.compute(bases.at(i)));
    double azimuth = reference_trajectory.azimuth(bases.at(i));
    double shift_length = shift_lengths.at(i);
    detail::to_point(shifted_points.back()).x += std::sin(azimuth) * shift_length;
    detail::to_point(shifted_points.back()).y -= std::cos(azimuth) * shift_length;
  }
  auto shifted_trajectory = reference_trajectory;
  bool valid = shifted_trajectory.build(shifted_points);
  if (!valid) {
    throw std::runtime_error(
      "Failed to build shifted trajectory");  // This Exception should not be thrown.
  }
  return shifted_trajectory;
}

template <class PointType>
trajectory::Trajectory<PointType> shift(
  const trajectory::Trajectory<PointType> & reference_trajectory,
  const ShiftInterval & shift_interval, const ShiftParameters & shift_parameters = {})
{
  return shift(reference_trajectory, std::vector{shift_interval}, shift_parameters);
}

template <class PointType>
trajectory::Trajectory<PointType> shift(
  const trajectory::Trajectory<PointType> & reference_trajectory, const double & lateral_shift)
{
  auto bases = reference_trajectory.get_internal_bases();
  std::vector<PointType> shifted_points;
  shifted_points.reserve(bases.size());
  for (const double & s : bases) {
    shifted_points.emplace_back(reference_trajectory.compute(s));
    const double azimuth = reference_trajectory.azimuth(s);
    detail::to_point(shifted_points.back()).x += std::sin(azimuth) * lateral_shift;
    detail::to_point(shifted_points.back()).y -= std::cos(azimuth) * lateral_shift;
  }
  auto shifted_trajectory = reference_trajectory;
  bool valid = shifted_trajectory.build(shifted_points);
  if (!valid) {
    throw std::runtime_error(
      "Failed to build shifted trajectory");  // This Exception should not be thrown.
  }
  return shifted_trajectory;
}

}  // namespace autoware::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__SHIFT_HPP_