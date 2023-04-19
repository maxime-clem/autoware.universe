// Copyright 2022 Tier IV, Inc.
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

#include "path_sampler/prepare_inputs.hpp"

#include "frenet_planner/structures.hpp"
#include "path_sampler/utils/geometry_utils.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"

#include <eigen3/unsupported/Eigen/Splines>

#include <boost/geometry/geometry.hpp>

#include <algorithm>
#include <list>
#include <memory>
#include <vector>

namespace path_sampler
{

void prepareConstraints(
  sampler_common::Constraints & constraints, const PredictedObjects & predicted_objects,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound)
{
  constraints.obstacle_polygons = sampler_common::MultiPolygon();
  for (auto & dynamic_obstacle_polygon :
       geometry_utils::PredictedObjectsToPolygons(predicted_objects)) {
    boost::geometry::correct(dynamic_obstacle_polygon);
    constraints.obstacle_polygons.push_back(dynamic_obstacle_polygon);
  }

  sampler_common::Polygon drivable_area_polygon;
  for (const auto & p : right_bound) drivable_area_polygon.outer().emplace_back(p.x, p.y);
  for (auto it = left_bound.rbegin(); it != left_bound.rend(); ++it)
    drivable_area_polygon.outer().emplace_back(it->x, it->y);
  drivable_area_polygon.outer().push_back(drivable_area_polygon.outer().front());
  constraints.drivable_polygons = {drivable_area_polygon};
  constraints.prefered_polygons = constraints.drivable_polygons;
}

frenet_planner::SamplingParameters prepareSamplingParameters(
  const sampler_common::State & initial_state,
  const double base_length, const sampler_common::transform::Spline2D & path_spline,
  const Parameters & params)
{
  frenet_planner::SamplingParameters sampling_parameters;
  sampling_parameters.resolution = params.sampling.resolution;
  (void)initial_state;
  (void)base_length;
  (void)path_spline;
  // TODO(Maxime): populate sampling parameters
  /*
  const auto max_s =
    path_spline.frenet({path.points.back().pose.position.x, path.points.back().pose.position.y}).s;
  for (const auto target_length : params.sampling.target_lengths) {
    const auto target_s =
      path_spline.frenet(initial_state.pose).s + std::max(0.0, target_length - base_length);
    // Prevent a target past the end of the reference path
    if (target_s < max_s)
      sampling_parameters.target_longitudinal_positions.push_back(target_s);
    else
      break;
  }
  // Stopping case
  if (sampling_parameters.target_longitudinal_positions.empty()) {
    sampling_parameters.target_longitudinal_positions = {max_s};
  }
  */
  return sampling_parameters;
}

sampler_common::transform::Spline2D preparePathSpline(
  const std::vector<TrajectoryPoint> & path, const bool smooth_path)
{
  std::vector<double> x;
  std::vector<double> y;
  if (smooth_path) {
    // TODO(Maxime CLEMENT): this version using Eigen::Spline often crashes
    constexpr auto spline_dim = 3;
    Eigen::MatrixXd control_points(path.size(), 2);
    for (auto i = 0lu; i < path.size(); ++i) {
      const auto & point = path[i];
      control_points(i, 0) = point.pose.position.x;
      control_points(i, 1) = point.pose.position.y;
    }
    control_points.transposeInPlace();
    const auto nb_knots = path.size() + spline_dim + 3;
    Eigen::RowVectorXd knots(nb_knots);
    constexpr auto repeat_endknots = 3lu;
    const auto knot_step = 1.0 / static_cast<double>(nb_knots - 2 * repeat_endknots);
    auto i = 0lu;
    for (; i < repeat_endknots; ++i) knots[i] = 0.0;
    for (; i < nb_knots - repeat_endknots; ++i) knots[i] = knots[i - 1] + knot_step;
    for (; i < nb_knots; ++i) knots[i] = 1.0;
    const auto spline = Eigen::Spline<double, 2, spline_dim>(knots, control_points);
    x.reserve(path.size());
    y.reserve(path.size());
    const auto t_step = 1 / static_cast<double>(path.size());
    for (auto t = 0.0; t < 1.0; t += t_step) {
      const auto p = spline(t);
      x.push_back(p.x());
      y.push_back(p.y());
    }
  } else {
    x.reserve(path.size());
    y.reserve(path.size());
    for (const auto & point : path) {
      x.push_back(point.pose.position.x);
      y.push_back(point.pose.position.y);
    }
  }
  return {x, y};
}
}  // namespace path_sampler
