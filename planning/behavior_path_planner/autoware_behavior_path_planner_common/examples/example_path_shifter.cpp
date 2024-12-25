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

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "matplotlibcpp17/pyplot.h"

#include <geometry_msgs/msg/pose.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <iostream>

tier4_planning_msgs::msg::PathPointWithLaneId create_path_point_with_lane_id(double x, double y)
{
  tier4_planning_msgs::msg::PathPointWithLaneId path_point_with_lane_id;
  path_point_with_lane_id.point.pose.position.x = x;
  path_point_with_lane_id.point.pose.position.y = y;
  return path_point_with_lane_id;
}

geometry_msgs::msg::Pose create_pose(double x, double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  return pose;
}

int main()
{
  tier4_planning_msgs::msg::PathWithLaneId path;
  for (double i = 0; i < 14.0; i += 0.5) {
    path.points.push_back(create_path_point_with_lane_id(i, 0.0));
  }

  autoware::behavior_path_planner::PathShifter path_shifter;

  path_shifter.setPath(path);
  path_shifter.setVelocity(3.0);
  path_shifter.setLateralAccelerationLimit(5.0);

  autoware::behavior_path_planner::ShiftLine shift_line1;
  shift_line1.start = create_pose(2, 0);
  shift_line1.end = create_pose(10, 0);
  shift_line1.end_shift_length = 1.0;

  autoware::behavior_path_planner::ShiftLine shift_line2;
  shift_line2.start = create_pose(5, 0);
  shift_line2.end = create_pose(7, 0);
  shift_line2.end_shift_length = 2.0;

  path_shifter.addShiftLine(shift_line1);
  path_shifter.addShiftLine(shift_line2);

  autoware::behavior_path_planner::ShiftedPath shifted_path;
  path_shifter.generate(&shifted_path);

  std::cout << "Shifted Path Size: " << shifted_path.path.points.size() << std::endl;
  std::cout << "Shifted Length Size: " << shifted_path.shift_length.size() << std::endl;
  for (const auto & shift_length : shifted_path.shift_length) {
    std::cout << shift_length << std::endl;
  }

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> x_shifted;
  std::vector<double> y_shifted;

  for (const auto & point : path.points) {
    x.push_back(point.point.pose.position.x);
    y.push_back(point.point.pose.position.y);
  }

  for (auto & point : shifted_path.path.points) {
    x_shifted.push_back(point.point.pose.position.x);
    y_shifted.push_back(point.point.pose.position.y);
  }

  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();
  plt.plot(Args(x, y), Kwargs("label"_a = "Original Path", "color"_a = "blue"));
  plt.scatter(Args(x, y), Kwargs("color"_a = "blue"));
  plt.plot(Args(x_shifted, y_shifted), Kwargs("label"_a = "Shifted Path", "color"_a = "red"));
  plt.scatter(Args(x_shifted, y_shifted), Kwargs("color"_a = "red"));
  plt.legend();
  plt.show();
  return 0;
}
