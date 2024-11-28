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

#include "start_planner_test_helper.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_path_start_planner_module/freespace_pull_out.hpp>
#include <autoware/behavior_path_start_planner_module/start_planner_module.hpp>
#include <autoware/behavior_path_start_planner_module/util.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::behavior_path_planner::FreespacePullOut;
using autoware::behavior_path_planner::StartPlannerParameters;
using autoware::test_utils::get_absolute_path_to_config;
using autoware_planning_msgs::msg::LaneletRoute;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
using autoware::behavior_path_planner::testing::StartPlannerTestHelper;
using autoware_planning_test_manager::utils::makeBehaviorRouteFromLaneId;

namespace autoware::behavior_path_planner
{

class TestFreespacePullOut : public ::testing::Test
{
public:
  std::optional<PullOutPath> call_plan(
    const Pose & start_pose, const Pose & goal_pose,
    const std::shared_ptr<const PlannerData> & planner_data, PlannerDebugData & planner_debug_data)
  {
    return freespace_pull_out_->plan(start_pose, goal_pose, planner_data, planner_debug_data);
  }

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ =
      rclcpp::Node::make_shared("freespace_pull_out", StartPlannerTestHelper::make_node_options());

    planner_data_ = std::make_shared<PlannerData>();
    planner_data_->init_parameters(*node_);

    initialize_freespace_pull_out_planner();
  }

  void TearDown() override { rclcpp::shutdown(); }

  // Member variables
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<FreespacePullOut> freespace_pull_out_;
  std::shared_ptr<PlannerData> planner_data_;

private:
  void initialize_freespace_pull_out_planner()
  {
    auto parameters = StartPlannerParameters::init(*node_);
    freespace_pull_out_ = std::make_shared<FreespacePullOut>(*node_, parameters);
  }
};

TEST_F(TestFreespacePullOut, GenerateValidFreespacePullOutPath)
{
  const auto start_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(299.796).y(303.529).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(-0.748629).w(
          0.662990));

  const auto goal_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(280.721).y(301.025).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.991718).w(
          0.128435));

  StartPlannerTestHelper::set_odometry(planner_data_, start_pose);
  StartPlannerTestHelper::set_route(planner_data_, 508, 720);
  StartPlannerTestHelper::set_costmap(planner_data_, start_pose, 0.3, 70.0, 70.0);

  // Plan the pull out path
  PlannerDebugData debug_data;
  auto result = call_plan(start_pose, goal_pose, planner_data_, debug_data);

  // Assert that a valid Freespace pull out path is generated
  ASSERT_TRUE(result.has_value()) << "Freespace pull out path generation failed.";
  // EXPECT_EQ(result->partial_paths.size(), 2UL)
  //   << "Freespace pull out path does not have the expected number of partial paths.";
  EXPECT_EQ(debug_data.conditions_evaluation.back(), "success")
    << "Freespace pull out path planning did not succeed.";
}

}  // namespace autoware::behavior_path_planner