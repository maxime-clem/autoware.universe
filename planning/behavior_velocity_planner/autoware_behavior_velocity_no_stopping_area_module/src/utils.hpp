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

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/no_stopping_area.hpp>
#include <rclcpp/logger.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::no_stopping_area
{
using PathIndexWithPose = std::pair<size_t, geometry_msgs::msg::Pose>;    // front index, pose
using PathIndexWithPoint2d = std::pair<size_t, universe_utils::Point2d>;  // front index, point2d
using PathIndexWithOffset = std::pair<size_t, double>;                    // front index, offset

struct PassJudge
{
  bool pass_judged = false;
  bool is_stoppable = true;
};

struct DebugData
{
  double base_link2front;
  std::vector<geometry_msgs::msg::Pose> stop_poses;
  geometry_msgs::msg::Pose first_stop_pose;
  std::vector<geometry_msgs::msg::Point> stuck_points;
  geometry_msgs::msg::Polygon stuck_vehicle_detect_area;
  geometry_msgs::msg::Polygon stop_line_detect_area;
};

/**
 * @brief check if the object is a vehicle (car, bus, truck, trailer, motorcycle)
 * @param object input object
 * @return true if the object is a vehicle
 */
bool is_vehicle_type(const autoware_perception_msgs::msg::PredictedObject & object);

/**
 * @brief insert stop point on ego path
 * @param path          original path
 * @param stop_point    stop line point on the lane
 */
void insert_stop_point(
  tier4_planning_msgs::msg::PathWithLaneId & path, const PathIndexWithPose & stop_point);

/**
 * @brief generate stop line from no stopping area polygons
 *          ________________
 * ------|--|--------------|--> ego path
 *  stop |  |     Area     |
 *  line |  L______________/
 * @param path input path
 * @param no_stopping_areas no stopping area polygons
 * @param ego_width [m] width of ego
 * @param stop_line_margin [m] margin to keep between the stop line and the no stopping areas
 **/
std::optional<universe_utils::LineString2d> generate_stop_line(
  const tier4_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::ConstPolygons3d & no_stopping_areas, const double ego_width,
  const double stop_line_margin);

/**
 * @brief Calculate if it's possible for ego-vehicle to stop before area consider jerk limit
 * @param [inout] pass_judge the pass judge decision to update
 * @param self_pose       ego-car pose
 * @param line_pose       stop line pose on the lane
 * @param planner_data planner data with ego pose, velocity, etc
 * @param logger ros logger
 * @return is stoppable in front of no stopping area
 */
bool is_stoppable(
  PassJudge & pass_judge, const geometry_msgs::msg::Pose & self_pose,
  const geometry_msgs::msg::Pose & line_pose, const PlannerData & planner_data,
  const rclcpp::Logger & logger);

/**
 * @brief Calculate the polygon of the path from the ego-car position to the end of the
 * no stopping lanelet (+ extra distance).
 * @param path           ego-car lane
 * @param ego_pose       ego-car pose
 * @param margin         margin from the end point of the ego-no stopping area lane
 * @param extra_dist     extra distance from the end point of the no stopping area lanelet
 * @return generated polygon
 */
Polygon2d generate_ego_no_stopping_area_lane_polygon(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose,
  const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem, const double margin,
  const double extra_dist, const double path_expand_width, const rclcpp::Logger & logger,
  rclcpp::Clock & clock);

/**
 * @brief Check if there is a stop line in "stop line detect area".
 * @param path            ego-car lane
 * @param poly            ego focusing area polygon
 * @return true if exists
 */
bool check_stop_lines_in_no_stopping_area(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const Polygon2d & poly,
  DebugData & debug_data);

/**
 * @brief Calculate the polygon of the path from the ego-car position to the end of the
 * no stopping lanelet (+ extra distance).
 * @param path                  ego-car lane
 * @param stop_line_margin      stop line margin from the stopping area lane
 * @return generated stop line
 */
std::optional<universe_utils::LineString2d> get_stop_line_geometry2d(
  const tier4_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem,
  const double stop_line_margin, const double stop_line_extend_length, const double vehicle_width);

}  // namespace autoware::behavior_velocity_planner::no_stopping_area

#endif  // UTILS_HPP_
