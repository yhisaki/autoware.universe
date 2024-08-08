#include <Eigen/Dense>
#include <autoware/motion_utils/trajectory_container/interpolator.hpp>
#include <autoware/motion_utils/trajectory_container/trajectory.hpp>

#include <matplotlibcpp17/pyplot.h>

tier4_planning_msgs::msg::PathPointWithLaneId path_with_lane_id(double x, double y, uint8_t lane_id)
{
  tier4_planning_msgs::msg::PathPointWithLaneId point;
  point.point.pose.position.x = x;
  point.point.pose.position.y = y;
  point.lane_ids.emplace_back(lane_id);
  return point;
}

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();

  autoware::motion_utils::trajectory_container::trajectory::TrajectoryContainer<
    tier4_planning_msgs::msg::PathPointWithLaneId>
    trajectory;

  std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> points{
    path_with_lane_id(0.00, 0.00, 0), path_with_lane_id(0.81, 1.68, 0),
    path_with_lane_id(1.65, 2.98, 0), path_with_lane_id(3.30, 4.01, 1),
    path_with_lane_id(4.70, 4.52, 1), path_with_lane_id(6.49, 5.20, 1),
    path_with_lane_id(8.11, 6.07, 1), path_with_lane_id(8.76, 7.23, 1),
    path_with_lane_id(9.36, 8.74, 1), path_with_lane_id(10.0, 10.0, 1)};

  trajectory
    .set_xy_interpolator(autoware::motion_utils::trajectory_container::interpolator::AkimaSpline())
    .build(points);

  std::vector<double> x;
  std::vector<double> y;

  for (double s = 0.0; s < trajectory.length(); s += 0.01) {
    auto point = trajectory.compute(s);
    x.push_back(point.point.pose.position.x);
    y.push_back(point.point.pose.position.y);
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = 5.0;
  pose.position.y = 5.0;

  double s = trajectory.closest(pose);

  std::cout << "Closest: " << s << std::endl;

  auto closest_pose = trajectory.compute(trajectory.closest(pose));
  plt.plot(Args(x, y));
  plt.axis(Args("equal"));
  plt.show();

  return 0;
}