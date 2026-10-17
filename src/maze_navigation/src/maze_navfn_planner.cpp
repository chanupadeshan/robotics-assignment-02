// Copyright 2026 maze_navigation contributors
// SPDX-License-Identifier: Apache-2.0
#include "maze_navigation/maze_navfn_planner.hpp"

#include <algorithm>
#include <cmath>
#include "pluginlib/class_list_macros.hpp"

namespace maze_navigation
{
nav_msgs::msg::Path MazeNavfnPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  if (cancel_checker()) {
    throw nav2_core::PlannerCancelled("Planning cancelled");
  }
  // Prevent a previous request's extraction count from qualifying this request.
  if (planner_) {
    planner_->npath = 0;
  }
  try {
    return NavfnPlanner::createPlan(start, goal, cancel_checker);
  } catch (const nav2_core::NoValidPathCouldBeFound &) {
    const int stock_limit = 4 * std::max(planner_->nx, planner_->ny);
    if (planner_->npath != stock_limit) {
      throw;  // A genuine no-route failure, not exhaustion of NavFn's fixed budget.
    }
    if (cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }
    // The stock half-cell step permits only 2 * map width of winding route.
    // Reuse its already computed Dijkstra/A* potential and exact collision costs.
    const int budget = std::max(stock_limit, 4 * planner_->ns);
    if (planner_->calcPath(budget) == 0) {
      throw;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = clock_->now();
    const auto * xs = planner_->getPathX();
    const auto * ys = planner_->getPathY();
    for (int i = planner_->getPathLen() - 1; i >= 0; --i) {
      if (i % 256 == 0 && cancel_checker()) {
        throw nav2_core::PlannerCancelled("Planning cancelled");
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      mapToWorld(xs[i], ys[i], pose.pose.position.x, pose.pose.position.y);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    smoothApproachToGoal(goal.pose, path);
    path.poses.back().pose.orientation = goal.pose.orientation;
    if (use_final_approach_orientation_ && path.poses.size() > 1) {
      const auto & a = path.poses[path.poses.size() - 2].pose.position;
      const auto & b = path.poses.back().pose.position;
      const double yaw = std::atan2(b.y - a.y, b.x - a.x);
      path.poses.back().pose.orientation.z = std::sin(yaw / 2.0);
      path.poses.back().pose.orientation.w = std::cos(yaw / 2.0);
    }
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 5000,
      "NavFn extended extraction succeeded: %zu points (stock limit %d)",
      path.poses.size(), stock_limit);
    return path;
  }
}
}  // namespace maze_navigation

PLUGINLIB_EXPORT_CLASS(maze_navigation::MazeNavfnPlanner, nav2_core::GlobalPlanner)
