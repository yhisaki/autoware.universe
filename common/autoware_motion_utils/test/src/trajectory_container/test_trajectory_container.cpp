#include <autoware/motion_utils/trajectory_container/trajectory.hpp>

#include <tier4_planning_msgs/msg/detail/path_point_with_lane_id__struct.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

tier4_planning_msgs::msg::PathPointWithLaneId path_with_lane_id(double x, double y, uint8_t lane_id)
{
  tier4_planning_msgs::msg::PathPointWithLaneId point;
  point.point.pose.position.x = x;
  point.point.pose.position.y = y;
  point.lane_ids.emplace_back(lane_id);
  return point;
}

TEST(TrajectoryContainer, test_name)
{
  using autoware::motion_utils::trajectory_container::trajectory::TrajectoryContainer;
  using tier4_planning_msgs::msg::PathPointWithLaneId;
  TrajectoryContainer<PathPointWithLaneId> trajectory;

  std::vector<PathPointWithLaneId> points{
    path_with_lane_id(0.08, 0.11, 0), path_with_lane_id(0.81, 1.68, 0),
    path_with_lane_id(1.65, 2.98, 0), path_with_lane_id(3.30, 4.01, 0),
    path_with_lane_id(4.70, 4.52, 0), path_with_lane_id(6.49, 5.20, 0),
    path_with_lane_id(8.11, 6.07, 0), path_with_lane_id(8.76, 7.23, 0),
    path_with_lane_id(9.36, 8.74, 0), path_with_lane_id(9.73, 9.80, 0)};

  trajectory.build(points);

  auto point = trajectory.compute(5.0);
}