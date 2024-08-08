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

#include <autoware/motion_utils/trajectory_container/interpolator.hpp>
#include <autoware/motion_utils/trajectory_container/trajectory.hpp>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <iostream>
#include <vector>

tier4_planning_msgs::msg::PathPointWithLaneId path_with_lane_id(double x, double y, uint8_t lane_id)
{
  tier4_planning_msgs::msg::PathPointWithLaneId point;
  point.point.pose.position.x = x;
  point.point.pose.position.y = y;
  point.lane_ids.emplace_back(lane_id);
  return point;
}

class TrajectoryContainerTest
: public ::testing::TestWithParam<std::tuple<std::string, std::string>>
{
public:
  autoware::motion_utils::trajectory_container::trajectory::TrajectoryContainer<
    tier4_planning_msgs::msg::PathPointWithLaneId>
    trajectory;

  void SetUp() override
  {
    std::string xy_interpolator_name = std::get<0>(GetParam());
    std::string lane_ids_interpolator_name = std::get<1>(GetParam());

    // Set XY interpolator
    if (xy_interpolator_name == "CubicSpline") {
      trajectory.set_xy_interpolator(
        autoware::motion_utils::trajectory_container::interpolator::CubicSpline());
    } else if (xy_interpolator_name == "AkimaSpline") {
      trajectory.set_xy_interpolator(
        autoware::motion_utils::trajectory_container::interpolator::AkimaSpline());
    } else if (xy_interpolator_name == "Linear") {
      trajectory.set_xy_interpolator(
        autoware::motion_utils::trajectory_container::interpolator::Linear());
    } else {
      FAIL() << "Unknown XY interpolator: " << xy_interpolator_name;
    }

    // Set lane IDs interpolator
    if (lane_ids_interpolator_name == "NearestNeighbor") {
      trajectory.set_lane_ids_interpolator(
        autoware::motion_utils::trajectory_container::interpolator::NearestNeighbor<
          std::vector<int64_t>>());
    } else if (lane_ids_interpolator_name == "ZeroOrderHold") {
      trajectory.set_lane_ids_interpolator(
        autoware::motion_utils::trajectory_container::interpolator::ZeroOrderHold<
          std::vector<int64_t>>());
    } else {
      FAIL() << "Unknown lane IDs interpolator: " << lane_ids_interpolator_name;
    }

    std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> points{
      path_with_lane_id(0.00, 0.00, 0), path_with_lane_id(0.81, 1.68, 0),
      path_with_lane_id(1.65, 2.98, 0), path_with_lane_id(3.30, 4.01, 1),
      path_with_lane_id(4.70, 4.52, 1), path_with_lane_id(6.49, 5.20, 1),
      path_with_lane_id(8.11, 6.07, 1), path_with_lane_id(8.76, 7.23, 1),
      path_with_lane_id(9.36, 8.74, 1), path_with_lane_id(10.0, 10.0, 1)};

    trajectory.build(points);
  }
};

TEST_P(TrajectoryContainerTest, compute)
{
  double length = trajectory.length();

  trajectory.longitudinal_velocity_mps(trajectory.length() / 3.0, trajectory.length()) = 10.0;

  auto point = trajectory.compute(length / 2.0);

  EXPECT_LT(0, point.point.pose.position.x);
  EXPECT_LT(point.point.pose.position.x, 10);

  EXPECT_LT(0, point.point.pose.position.y);
  EXPECT_LT(point.point.pose.position.y, 10);

  EXPECT_EQ(1, point.lane_ids[0]);
}

TEST_P(TrajectoryContainerTest, manipulate_velocity)
{
  trajectory.longitudinal_velocity_mps = 10.0;
  trajectory.longitudinal_velocity_mps(trajectory.length() / 3, 2.0 * trajectory.length() / 3) =
    5.0;
  auto point1 = trajectory.compute(0.0);
  auto point2 = trajectory.compute(trajectory.length() / 2.0);
  auto point3 = trajectory.compute(trajectory.length());

  EXPECT_EQ(10.0, point1.point.longitudinal_velocity_mps);
  EXPECT_EQ(5.0, point2.point.longitudinal_velocity_mps);
  EXPECT_EQ(10.0, point3.point.longitudinal_velocity_mps);
}

TEST_P(TrajectoryContainerTest, direction)
{
  double dir = trajectory.direction(0.0);
  EXPECT_LT(0, dir);
  EXPECT_LT(dir, M_PI / 2);
}

TEST_P(TrajectoryContainerTest, curvature)
{
  double curv = trajectory.curvature(0.0);
  EXPECT_LT(-1.0, curv);
  EXPECT_LT(curv, 1.0);
}

TEST_P(TrajectoryContainerTest, restore)
{
  using namespace autoware::motion_utils::trajectory_container::trajectory;  // NOLINT
  trajectory.longitudinal_velocity_mps(4.0, trajectory.length()) = 5.0;

  {
    auto points =
      static_cast<TrajectoryContainer<geometry_msgs::msg::Point> &>(trajectory).restore();
    EXPECT_EQ(10, points.size());
  }

  {
    auto points =
      static_cast<TrajectoryContainer<geometry_msgs::msg::Pose> &>(trajectory).restore();
    EXPECT_EQ(10, points.size());
  }

  {
    auto points =
      static_cast<TrajectoryContainer<autoware_planning_msgs::msg::PathPoint> &>(trajectory)
        .restore();
    EXPECT_EQ(11, points.size());
  }

  {
    auto points = trajectory.restore();
    EXPECT_EQ(11, points.size());
  }
}

TEST_P(TrajectoryContainerTest, crossed)
{
  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.0;
  pose1.position.y = 10.0;
  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 10.0;
  pose2.position.y = 0.0;

  auto crossed_point = trajectory.crossed(pose1, pose2);
  EXPECT_TRUE(crossed_point.has_value());

  EXPECT_LT(0.0, *crossed_point);
  EXPECT_LT(*crossed_point, trajectory.length());
}

TEST_P(TrajectoryContainerTest, closest)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 5.0;
  pose.position.y = 5.0;

  std::cerr << "Closest: " << trajectory.closest(pose) << std::endl;

  auto closest_pose = trajectory.compute(trajectory.closest(pose));

  double distance = std::hypot(
    closest_pose.point.pose.position.x - pose.position.x,
    closest_pose.point.pose.position.y - pose.position.y);

  EXPECT_LT(distance, 3.0);
}

INSTANTIATE_TEST_CASE_P(
  TrajectoryInterpolators, TrajectoryContainerTest,
  ::testing::Combine(
    ::testing::Values("CubicSpline", "AkimaSpline", "Linear"),
    ::testing::Values("ZeroOrderHold", "NearestNeighbor")));