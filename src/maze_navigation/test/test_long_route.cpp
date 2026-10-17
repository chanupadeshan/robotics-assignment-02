// Copyright 2026 maze_navigation contributors
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include "gtest/gtest.h"
#include "maze_navigation/maze_navfn_planner.hpp"

class PlannerHarness : public maze_navigation::MazeNavfnPlanner
{
public:
  PlannerHarness()
  {
    std::ifstream file(COSTMAP_FIXTURE, std::ios::binary);
    std::string magic;
    int width, height, maximum;
    file >> magic >> width >> height >> maximum;
    file.get();
    if (!file || magic != "P5" || width != 128 || height != 128 || maximum != 255) {
      throw std::runtime_error("Invalid costmap fixture");
    }
    owned_map_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
      width, height, 0.025, -1.6, -1.6, 0);
    file.read(reinterpret_cast<char *>(owned_map_->getCharMap()), width * height);
    if (!file) {throw std::runtime_error("Truncated fixture");}
    costmap_ = owned_map_.get();
    planner_ = std::make_unique<nav2_navfn_planner::NavFn>(width, height);
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    global_frame_ = "map";
    name_ = "GridBased";
    tolerance_ = 0.05;
    allow_unknown_ = false;
    use_astar_ = false;
    use_final_approach_orientation_ = false;
  }

  nav_msgs::msg::Path stock(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    return NavfnPlanner::createPlan(start, goal, [] {return false;});
  }

  int extracted() const {return planner_->npath;}

  void blockRoute()
  {
    // Seal every cell except a small free patch at the start and goal.
    std::fill(costmap_->getCharMap(), costmap_->getCharMap() + 128 * 128, 254);
    for (int y = 10; y < 20; ++y) {
      for (int x = 40; x < 50; ++x) {costmap_->setCost(x, y, 0);}
    }
    for (int y = 110; y < 120; ++y) {
      for (int x = 110; x < 120; ++x) {costmap_->setCost(x, y, 0);}
    }
  }

private:
  std::unique_ptr<nav2_costmap_2d::Costmap2D> owned_map_;
};

geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(0.5 * 1.57079632679);
  p.pose.orientation.w = std::cos(0.5 * 1.57079632679);
  return p;
}

TEST(MazeNavfn, LongConnectedRouteExceedsStockBudget)
{
  PlannerHarness planner;
  const auto start = pose(-0.460687, -1.245224);
  const auto goal = pose(1.25, 1.25);
  EXPECT_THROW(planner.stock(start, goal), nav2_core::NoValidPathCouldBeFound);
  EXPECT_EQ(planner.extracted(), 512);
  const auto result = planner.createPlan(start, goal, [] {return false;});
  ASSERT_GT(result.poses.size(), 512u);
  EXPECT_NEAR(result.poses.back().pose.position.x, 1.25, 0.001);
  EXPECT_NEAR(result.poses.back().pose.position.y, 1.25, 0.001);
  EXPECT_EQ(result.poses.back().pose.orientation, goal.pose.orientation);
}

TEST(MazeNavfn, ShortRouteRetainsStockBehavior)
{
  PlannerHarness planner;
  const auto start = pose(-0.460687, -1.245224);
  const auto goal = pose(-0.75, -1.25);
  const auto expected = planner.stock(start, goal);
  const auto actual = planner.createPlan(start, goal, [] {return false;});
  ASSERT_EQ(expected.poses.size(), actual.poses.size());
  for (size_t i = 0; i < expected.poses.size(); ++i) {
    EXPECT_EQ(expected.poses[i].pose, actual.poses[i].pose);
  }
}

TEST(MazeNavfn, DisconnectedGoalRemainsRejected)
{
  PlannerHarness planner;
  // A prior exhausted extraction must not turn a later disconnected map into a path.
  ASSERT_GT(planner.createPlan(
      pose(-0.460687, -1.245224), pose(1.25, 1.25), [] {return false;}).poses.size(), 512u);
  planner.blockRoute();
  EXPECT_THROW(
    planner.createPlan(pose(-0.46, -1.245), pose(1.25, 1.25), [] {return false;}),
    nav2_core::NoValidPathCouldBeFound);
}

TEST(MazeNavfn, CancellationIsRespected)
{
  PlannerHarness planner;
  EXPECT_THROW(
    planner.createPlan(pose(-0.46, -1.245), pose(1.25, 1.25), [] {return true;}),
    nav2_core::PlannerCancelled);
}

TEST(MazeNavfn, CancellationBeforeExtendedExtractionIsRespected)
{
  PlannerHarness planner;
  EXPECT_THROW(
    planner.createPlan(pose(-0.460687, -1.245224), pose(1.25, 1.25),
      [&planner] {return planner.extracted() == 512;}),
    nav2_core::PlannerCancelled);
}
