// Copyright 2024 Tier IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__UTILS__TYPES_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__UTILS__TYPES_HPP_

#include "lanelet2_core/primitives/Point.h"

#include <Eigen/Dense>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <Eigen/src/Core/util/Meta.h>

#include <set>
#include <vector>

namespace autoware::motion_utils::trajectory_container::detail
{
inline geometry_msgs::msg::Point to_point(const geometry_msgs::msg::Point & p)
{
  return p;
}
inline geometry_msgs::msg::Point to_point(const geometry_msgs::msg::Pose & p)
{
  return p.position;
}
inline geometry_msgs::msg::Point to_point(const Eigen::Ref<const Eigen::Vector2d> & p)
{
  geometry_msgs::msg::Point point;
  point.x = p(0);
  point.y = p(1);
  return point;
}
inline geometry_msgs::msg::Point to_point(const autoware_planning_msgs::msg::PathPoint & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.pose.position.x;
  point.y = p.pose.position.y;
  return point;
}
inline geometry_msgs::msg::Point to_point(const tier4_planning_msgs::msg::PathPointWithLaneId & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.point.pose.position.x;
  point.y = p.point.pose.position.y;
  return point;
}
inline geometry_msgs::msg::Point to_point(const lanelet::BasicPoint2d & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.x();
  point.y = p.y();
  return point;
}
inline geometry_msgs::msg::Point to_point(const lanelet::ConstPoint3d & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.x();
  point.y = p.y();
  return point;
}

/**
 * @brief Merge multiple vectors into one, keeping only unique elements.
 * @tparam Vectors Variadic template parameter for vector types.
 * @param vectors Vectors to be merged.
 * @return Merged vector with unique elements.
 */
template <typename... Vectors>
std::vector<double> merge_vectors(const Vectors &... vectors)
{
  std::set<double> unique_elements;

  // Helper function to insert elements into the set
  auto insert_elements = [&unique_elements](const auto & vec) {
    unique_elements.insert(vec.begin(), vec.end());
  };

  // Expand the parameter pack and insert elements from each vector
  (insert_elements(vectors), ...);

  // Convert the set back to a vector
  std::vector<double> result(unique_elements.begin(), unique_elements.end());

  return result;
}

}  // namespace autoware::motion_utils::trajectory_container::detail

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__UTILS__TYPES_HPP_
