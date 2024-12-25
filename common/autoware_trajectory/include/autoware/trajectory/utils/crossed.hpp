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

#ifndef AUTOWARE__TRAJECTORY__UTILS__CROSSED_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__CROSSED_HPP_
#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"

#include <Eigen/Core>

#include <boost/concept/assert.hpp>
#include <boost/concept_check.hpp>
#include <boost/iterator/iterator_concepts.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <vector>

namespace autoware::trajectory
{

namespace detail::concept
{
  template <class T>
  struct XYPointConcept
  {
    void constraints()
    {
      boost::function_requires<boost::ConvertibleConcept<decltype(x.x()), double>>();
      boost::function_requires<boost::ConvertibleConcept<decltype(x.y()), double>>();
    }
    T x;
  };

  // Container concept definition
  template <class Container>
  struct XYContainerConcept
  {
    using value_type = typename Container::value_type;
    using iterator = typename Container::iterator;

    void constraints()
    {
      // Check if it's a container
      boost::function_requires<boost::ContainerConcept<Container>>();

      // Check if elements have x() and y()
      boost::function_requires<XYPointConcept<value_type>>();

      // Additional container requirements
      iterator it = container.begin();
      iterator end = container.end();
      value_type & item = *it;

      // Check element access
      double x = item.x();
      double y = item.y();

      boost::ignore_unused_variable_warning(x);
      boost::ignore_unused_variable_warning(y);
      boost::ignore_unused_variable_warning(it);
      boost::ignore_unused_variable_warning(end);
    }

    Container container;
  };
}  // namespace detail::concept

namespace detail::impl
{
std::vector<double> crossed_with_constraint_impl(
  const std::function<Eigen::Vector2d(const double & s)> & trajectory_compute,
  const std::vector<double> & bases,  //
  const std::vector<Eigen::Vector2d> & linestrings_start,
  const std::vector<Eigen::Vector2d> & linestrings_end,
  const std::function<bool(const double &)> & constraint);
}  // namespace detail::impl

template <class TrajectoryPointType, class LineStringType>
[[nodiscard]] std::vector<double> crossed_with_constraint(
  const trajectory::Trajectory<TrajectoryPointType> & trajectory, const LineStringType & linestring,
  const std::function<bool(const TrajectoryPointType &)> & constraint)
{
  using autoware::trajectory::detail::to_point;

  std::function<Eigen::Vector2d(const double & s)> trajectory_compute =
    [&trajectory](const double & s) {
      TrajectoryPointType point = trajectory.compute(s);
      Eigen::Vector2d result;
      result << to_point(point).x, to_point(point).y;
      return result;
    };

  std::vector<Eigen::Vector2d> linestrings_start;
  std::vector<Eigen::Vector2d> linestrings_end;

  for (size_t i = 1; i < linestring.size(); ++i) {
    linestrings_start.push_back(
      Eigen::Vector2d(linestring.at(i - 1).x(), linestring.at(i - 1).y()));
    linestrings_end.push_back(Eigen::Vector2d(linestring.at(i).x(), linestring.at(i).y()));
  }

  return detail::impl::crossed_with_constraint_impl(
    trajectory_compute, trajectory.get_internal_bases(), linestrings_start, linestrings_end,
    constraint);
}

template <class TrajectoryPointType, class LineStringType>
[[nodiscard]] std::vector<double> crossed(
  const trajectory::Trajectory<TrajectoryPointType> & trajectory, const LineStringType & linestring)
{
  // BOOST_CONCEPT_ASSERT((detail::concept ::XYContainerConcept<LineStringType>));
  return crossed_with_constraint(
    trajectory, linestring, [](const TrajectoryPointType &) { return true; });
}

}  // namespace autoware::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__CROSSED_HPP_
