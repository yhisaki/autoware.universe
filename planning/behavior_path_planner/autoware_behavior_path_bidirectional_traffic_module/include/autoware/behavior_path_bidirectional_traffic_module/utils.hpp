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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__UTILS_HPP_

#include <autoware/trajectory/forward.hpp>
#include <autoware/trajectory/utils/find_intervals.hpp>

#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>

#include <unordered_set>
#include <vector>
namespace autoware::behavior_path_planner
{

class BidirectionalLanes
{
  std::vector<std::pair<lanelet::ConstLanelet, lanelet::ConstLanelet>> bidirectional_lanes_;
  std::unordered_set<lanelet::Id> all_ids_set_;

public:
  void add(const lanelet::ConstLanelet & lanelet_a, const lanelet::ConstLanelet & lanelet_b);
  [[nodiscard]] bool is_bidirectional_lanes(const lanelet::Ids & lane_ids) const;
};

bool is_bidirectional_lanes(
  const lanelet::ConstLanelet & lanelet_a, const lanelet::ConstLanelet & lanelet_b);

BidirectionalLanes search_bidirectional_lane_from_map(const lanelet::LaneletMap & map);

trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>
shift_trajectory_for_keep_left(
  const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const std::vector<trajectory::Interval> bidirectional_lane_intervals_in_trajectory,
  const double & keep_left_length_from_center,
  const double & distance_to_shift_for_enter_bidirectional_lane,
  const double & distance_to_shift_for_exit_bidirectional_lane);
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__UTILS_HPP_
