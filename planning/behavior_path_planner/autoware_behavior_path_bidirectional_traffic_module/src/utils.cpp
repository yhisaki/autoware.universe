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

#include "autoware/behavior_path_bidirectional_traffic_module/utils.hpp"

#include "autoware/trajectory/path_point_with_lane_id.hpp"

#include <autoware/trajectory/utils/shift.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>

namespace autoware::behavior_path_planner
{

void BidirectionalLanes::add(
  const lanelet::ConstLanelet & lanelet_a, const lanelet::ConstLanelet & lanelet_b)
{
  bidirectional_lanes_.emplace_back(lanelet_a, lanelet_b);
  all_ids_set_.insert(lanelet_a.id());
  all_ids_set_.insert(lanelet_b.id());
}

bool BidirectionalLanes::is_bidirectional_lanes(const lanelet::Ids & lane_ids) const
{
  return std::any_of(lane_ids.begin(), lane_ids.end(), [&](const lanelet::Id & id) {
    return all_ids_set_.count(id);
  });
}

bool is_bidirectional_lanes(
  const lanelet::ConstLanelet & lanelet_a, const lanelet::ConstLanelet & lanelet_b)
{
  return (lanelet_a.leftBound().id() == lanelet_b.rightBound().id()) &&
         (lanelet_a.rightBound().id() == lanelet_b.leftBound().id());
}

BidirectionalLanes search_bidirectional_lane_from_map(const lanelet::LaneletMap & map)
{
  BidirectionalLanes bidirectional_lanes;

  // Lambda function to check and add bidirectional lane pairs
  auto add_bidirectional_lane = [&bidirectional_lanes](
                                  const lanelet::ConstLanelet & lanelet_a,
                                  const lanelet::ConstLanelet & lanelet_b) {
    if (is_bidirectional_lanes(lanelet_a, lanelet_b)) {
      bidirectional_lanes.add(lanelet_a, lanelet_b);
    }
  };

  // Iterate over all lanelet pairs without duplication
  for (auto it_a = map.laneletLayer.begin(); it_a != map.laneletLayer.end(); ++it_a) {
    std::for_each(
      std::next(it_a), map.laneletLayer.end(),
      [&](const lanelet::ConstLanelet & lanelet_b) { add_bidirectional_lane(*it_a, lanelet_b); });
  }
  return bidirectional_lanes;
}

trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>
shift_trajectory_for_keep_left(
  const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const std::vector<trajectory::Interval> & bidirectional_lane_intervals_in_trajectory,
  const double & keep_left_length_from_center,
  const double & distance_to_shift_for_enter_bidirectional_lane,
  const double & distance_to_shift_for_exit_bidirectional_lane)
{
  auto shifted_trajectory = trajectory;

  auto is_inside_trajectory = [&trajectory](const double & x) {
    return 0 <= x && x <= trajectory.length();
  };

  for (const auto & bidirectional_lane_interval : bidirectional_lane_intervals_in_trajectory) {
    const double shift_start_for_enter_bidirectional_lane =
      bidirectional_lane_interval.start - distance_to_shift_for_enter_bidirectional_lane;
    const double shift_end_for_exit_bidirectional_lane =
      bidirectional_lane_interval.end + distance_to_shift_for_exit_bidirectional_lane;
    if (
      is_inside_trajectory(shift_start_for_enter_bidirectional_lane) &&
      !is_inside_trajectory(shift_end_for_exit_bidirectional_lane)) {
      trajectory::ShiftInterval shift_interval{
        shift_start_for_enter_bidirectional_lane, bidirectional_lane_interval.start,
        keep_left_length_from_center};
      shifted_trajectory = trajectory::shift(shifted_trajectory, shift_interval);
    }
    if (
      !is_inside_trajectory(shift_start_for_enter_bidirectional_lane) &&
      !is_inside_trajectory(shift_end_for_exit_bidirectional_lane)) {
      shifted_trajectory = trajectory::shift(shifted_trajectory, keep_left_length_from_center);
    }
    if (
      is_inside_trajectory(shift_start_for_enter_bidirectional_lane) &&
      is_inside_trajectory(shift_end_for_exit_bidirectional_lane)) {
      trajectory::ShiftInterval shift_interval1{
        shift_start_for_enter_bidirectional_lane, bidirectional_lane_interval.start,
        keep_left_length_from_center};
      trajectory::ShiftInterval shift_interval2{
        bidirectional_lane_interval.end, shift_end_for_exit_bidirectional_lane,
        -keep_left_length_from_center};
      shifted_trajectory =
        trajectory::shift(shifted_trajectory, {shift_interval1, shift_interval2});
    }
    if (
      !is_inside_trajectory(shift_start_for_enter_bidirectional_lane) &&
      is_inside_trajectory(shift_end_for_exit_bidirectional_lane)) {
      trajectory::ShiftInterval shift_interval{
        bidirectional_lane_interval.end, shift_end_for_exit_bidirectional_lane,
        -keep_left_length_from_center};
      shifted_trajectory = trajectory::shift(shifted_trajectory, keep_left_length_from_center);
      shifted_trajectory = trajectory::shift(shifted_trajectory, shift_interval);
    }
  }

  return shifted_trajectory;
}

}  // namespace autoware::behavior_path_planner
