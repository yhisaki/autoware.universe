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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__ZERO_ORDER_HOLD_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__ZERO_ORDER_HOLD_HPP_

#include "autoware/motion_utils/trajectory_container/interpolator/interpolator.hpp"

#include <vector>

namespace autoware::motion_utils::trajectory_container::interpolator
{

/**
 * @brief Template class for zero order hold interpolation.
 *
 * This class provides methods to perform zero order hold interpolation on a set of data points.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class ZeroOrderHold;

namespace detail
{

/**
 * @brief Base class for zero order hold interpolation.
 *
 * This class implements the core functionality for zero order hold interpolation.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class ZeroOrderHoldCommonImpl : public Interpolator<T>
{
protected:
  std::vector<T> values;  ///< Interpolation values.

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  [[nodiscard]] T compute_impl(const double & s) const override
  {
    int idx = this->get_index(s, true);
    return this->values[idx];
  }

  /**
   * @brief Build the interpolator with the given values.
   *
   * @param values The values to interpolate.
   */
  void build_impl(const std::vector<T> & values) override { this->values = values; }

public:
  /**
   * @brief Default constructor.
   */
  ZeroOrderHoldCommonImpl() = default;

  /**
   * @brief Get the minimum number of required points for the interpolator.
   */
  [[nodiscard]] size_t minimum_required_points() const override { return 2; }
};

}  // namespace detail

/**
 * @brief Template class for zero order hold interpolation.
 *
 * This class provides the interface for zero order hold interpolation.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class ZeroOrderHold : public detail::ZeroOrderHoldCommonImpl<T>
{
};

/**
 * @brief Specialization of ZeroOrderHold for double values.
 *
 * This class provides methods to perform zero order hold interpolation on double values.
 */
template <>
class ZeroOrderHold<double> : public detail::ZeroOrderHoldCommonImpl<double>
{
private:
  /**
   * @brief Compute the first derivative at the given point.
   *
   * @param s The point at which to compute the first derivative.
   * @return The first derivative.
   */
  [[nodiscard]] double compute_first_derivative_impl(const double &) const override { return 0.0; }

  /**
   * @brief Compute the second derivative at the given point.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  [[nodiscard]] double compute_second_derivative_impl(const double &) const override { return 0.0; }
};

}  // namespace autoware::motion_utils::trajectory_container::interpolator

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__ZERO_ORDER_HOLD_HPP_
