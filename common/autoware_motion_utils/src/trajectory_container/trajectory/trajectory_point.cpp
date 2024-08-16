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

#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_point.hpp"

#include "autoware/motion_utils/trajectory_container/interpolator/cubic_spline.hpp"
#include "autoware/motion_utils/trajectory_container/interpolator/linear.hpp"

#include <fmt/format.h>

#include <cmath>

namespace autoware::motion_utils::trajectory_container::trajectory
{
TrajectoryContainer<geometry_msgs::msg::Point>::TrajectoryContainer()
: start_(std::nan("")), end_(std::nan(""))
{
  set_xy_interpolator(interpolator::CubicSpline());
  set_z_interpolator(interpolator::Linear());
}

TrajectoryContainer<geometry_msgs::msg::Point> & TrajectoryContainer<
  geometry_msgs::msg::Point>::build(const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;

  axis_.resize(static_cast<Eigen::Index>(points.size()));
  axis_(0) = 0.0;
  xs.emplace_back(points[0].x);
  ys.emplace_back(points[0].y);
  zs.emplace_back(points[0].z);

  for (size_t i = 1; i < points.size(); ++i) {
    Eigen::Vector2d p0(points[i - 1].x, points[i - 1].y);
    Eigen::Vector2d p1(points[i].x, points[i].y);
    axis_(static_cast<Eigen::Index>(i)) =
      axis_(static_cast<Eigen::Index>(i - 1)) + (p1 - p0).norm();
    xs.emplace_back(points[i].x);
    ys.emplace_back(points[i].y);
    zs.emplace_back(points[i].z);
  }

  start_ = axis_(0);
  end_ = axis_(axis_.size() - 1);

  x_interpolator_->build(axis_, xs);
  y_interpolator_->build(axis_, ys);
  z_interpolator_->build(axis_, zs);

  return *this;
}

void TrajectoryContainer<geometry_msgs::msg::Point>::validate_s(const double & s) const
{
  if (s < start_ || s > end_) {
    throw std::out_of_range(fmt::format(
      "The arc length {:.2f} is out of the trajectory range [{:.2f}, {:.2f}]", s, start_, end_));
  }
}

double TrajectoryContainer<geometry_msgs::msg::Point>::length() const
{
  return end_ - start_;
}

geometry_msgs::msg::Point TrajectoryContainer<geometry_msgs::msg::Point>::compute(
  const double & s) const
{
  validate_s(s);
  geometry_msgs::msg::Point result;
  result.x = x_interpolator_->compute(s + start_);
  result.y = y_interpolator_->compute(s + start_);
  result.z = z_interpolator_->compute(s + start_);
  return result;
}

double TrajectoryContainer<geometry_msgs::msg::Point>::direction(const double & s) const
{
  validate_s(s);
  double dx = x_interpolator_->compute_first_derivative(s + start_);
  double dy = y_interpolator_->compute_first_derivative(s + start_);
  return std::atan2(dy, dx);
}

double TrajectoryContainer<geometry_msgs::msg::Point>::curvature(const double & s) const
{
  validate_s(s);
  double dx = x_interpolator_->compute_first_derivative(s + start_);
  double ddx = x_interpolator_->compute_second_derivative(s + start_);
  double dy = y_interpolator_->compute_first_derivative(s + start_);
  double ddy = y_interpolator_->compute_second_derivative(s + start_);
  return std::abs(dx * ddy - dy * ddx) / std::pow(dx * dx + dy * dy, 1.5);
}

std::vector<geometry_msgs::msg::Point> TrajectoryContainer<geometry_msgs::msg::Point>::restore()
  const
{
  std::vector<geometry_msgs::msg::Point> points(axis_.size());
  std::transform(axis_.begin(), axis_.end(), points.begin(), [this](const auto & s) {
    geometry_msgs::msg::Point p;
    p.x = x_interpolator_->compute(s);
    p.y = y_interpolator_->compute(s);
    p.z = z_interpolator_->compute(s);
    return p;
  });
  return points;
}

}  // namespace autoware::motion_utils::trajectory_container::trajectory
