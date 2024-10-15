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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::detection_area
{

/// @brief get the extended stop line of the given detection area
/// @param [in] detection_area detection area
/// @param [in] extend_length [m] extension length to add on each edge of the stop line
/// @return extended stop line
universe_utils::LineString2d get_stop_line_geometry2d(
  const lanelet::autoware::DetectionArea & detection_area, const double extend_length);

std::vector<geometry_msgs::msg::Point> get_obstacle_points(
  const lanelet::ConstPolygons3d & detection_areas, const pcl::PointCloud<pcl::PointXYZ> & points);

bool can_clear_stop_state(
  const std::shared_ptr<const rclcpp::Time> & last_obstacle_found_time, const rclcpp::Time & now,
  const double state_clear_time);

bool has_enough_braking_distance(
  const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose,
  const double pass_judge_line_distance, const double current_velocity);

// calc smallest enclosing circle with average O(N) algorithm
// reference:
// https://erickimphotography.com/blog/wp-content/uploads/2018/09/Computational-Geometry-Algorithms-and-Applications-3rd-Ed.pdf
std::pair<lanelet::BasicPoint2d, double> get_smallest_enclosing_circle(
  const lanelet::ConstPolygon2d & poly);
}  // namespace autoware::behavior_velocity_planner::detection_area

#endif  // UTILS_HPP_
