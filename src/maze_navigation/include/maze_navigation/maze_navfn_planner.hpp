// Copyright 2026 maze_navigation contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef MAZE_NAVIGATION__MAZE_NAVFN_PLANNER_HPP_
#define MAZE_NAVIGATION__MAZE_NAVFN_PLANNER_HPP_

#include <functional>
#include "nav2_navfn_planner/navfn_planner.hpp"

namespace maze_navigation
{
// Keep stock NavFn behavior; extend only a demonstrably exhausted extraction budget.
class MazeNavfnPlanner : public nav2_navfn_planner::NavfnPlanner
{
public:
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;
};
}  // namespace maze_navigation
#endif  // MAZE_NAVIGATION__MAZE_NAVFN_PLANNER_HPP_
