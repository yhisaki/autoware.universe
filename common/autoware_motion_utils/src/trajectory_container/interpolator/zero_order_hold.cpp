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

#include "autoware/motion_utils/trajectory_container/interpolator/zero_order_hold.hpp"

#include <Eigen/Dense>

#include <vector>

namespace autoware::motion_utils::trajectory_container::interpolator::detail
{

template <typename T>
T ZeroOrderHoldCommonImpl<T>::compute_impl(const double & s) const
{
  int idx = this->get_index(s, true);
  return this->values[idx];
}

template <typename T>
void ZeroOrderHoldCommonImpl<T>::build_impl(const std::vector<T> & values)
{
  this->values = values;
}

// Explicit template instantiation
template class ZeroOrderHoldCommonImpl<double>;
template class ZeroOrderHoldCommonImpl<std::vector<int64_t>>;
template class ZeroOrderHoldCommonImpl<std::vector<std::vector<int64_t>>>;

}  // namespace autoware::motion_utils::trajectory_container::interpolator::detail
