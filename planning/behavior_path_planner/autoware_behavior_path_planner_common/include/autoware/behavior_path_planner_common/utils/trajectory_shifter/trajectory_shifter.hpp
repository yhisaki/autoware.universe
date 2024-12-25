// Copyright 2021 Tier IV, Inc.
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
#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__TRAJECTORY_SHIFTER__TRAJECTORY_SHIFTER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__TRAJECTORY_SHIFTER__TRAJECTORY_SHIFTER_HPP_

#include "autoware/trajectory/forward.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

struct ShiftInterval
{
  double start{0.0};
  double end{0.0};
  double start_shift{0.0};
  double end_shift{0.0};
};

std::pair<std::vector<double>, std::vector<double>> calc_base_lengths(
  const double & arclength, const double & shift_length, const bool & offset_back,
  const double & velocity, const double & longitudinal_acc, const double & lateral_acc_limit);

template <class PointType>
class TrajectoryShifterBase
{
public:
  virtual autoware::trajectory::Trajectory<PointType> operator()(
    const autoware::trajectory::Trajectory<PointType> & trajectory,
    const std::vector<ShiftInterval> & shift_intervals) = 0;
};

template <class PointType>
class SplineShifter : public TrajectoryShifterBase<PointType>
{
  using TrajectoryType = autoware::trajectory::Trajectory<PointType>;

private:
  const double velocity_;
  const double longitudinal_acc_;
  const double lateral_acc_limit_;

  TrajectoryType apply_single_shift(
    const TrajectoryType & reference_trajectory, const TrajectoryType /* shifted_trajectory */,
    const ShiftInterval & /* shift_interval */)
  {
    auto reference_trajectory_bases = reference_trajectory.get_internal_bases();

    return reference_trajectory;
  }

public:
  SplineShifter(double velocity, double longitudinal_acc, double lateral_acc_limit)
  : velocity_(velocity), longitudinal_acc_(longitudinal_acc), lateral_acc_limit_(lateral_acc_limit)
  {
  }

  TrajectoryType operator()(
    const TrajectoryType & reference_trajectory, const TrajectoryType /* shifted_trajectory */,
    const ShiftInterval & /* shift_interval */)
  {
    auto reference_trajectory_bases = reference_trajectory.get_internal_bases();

    return reference_trajectory;
  }

  TrajectoryType operator()(
    const TrajectoryType & reference_trajectory,
    const std::vector<ShiftInterval> & shift_intervals) override
  {
    TrajectoryType shifted_trajectory = reference_trajectory;
    for (const auto & shift_interval : shift_intervals) {
      shifted_trajectory =
        apply_single_shift(reference_trajectory, shifted_trajectory, shift_interval);
    }
    return shifted_trajectory;
  }
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__TRAJECTORY_SHIFTER__TRAJECTORY_SHIFTER_HPP_
