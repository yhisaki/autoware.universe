#include "autoware/behavior_path_planner_common/utils/trajectory_shifter/trajectory_shifter.hpp"
#include "autoware/trajectory/path_point.hpp"

autoware_planning_msgs::msg::PathPoint path_point(double x, double y)
{
  autoware_planning_msgs::msg::PathPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.longitudinal_velocity_mps = 1.0;
  return p;
}

int main()
{
  using TrajectoryShifterType =
    autoware::behavior_path_planner::SplineShifter<autoware_planning_msgs::msg::PathPoint>;

  TrajectoryShifterType shifter(1.0, 1.0, 1.0);
}
