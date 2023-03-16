// Copyright 2023 Tier IV, Inc.
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

#ifndef POTENTIAL_FIELDS__POTENTIAL_FIELDS_HPP_
#define POTENTIAL_FIELDS__POTENTIAL_FIELDS_HPP_

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>

#include <algorithm>

namespace potential_fields
{
double get_simplified_centerline_pf(
  const double x, const double y, const tier4_autoware_utils::LineString2d & centerline)
{
  constexpr auto lambda_square = 1.0;  // TODO(Lin): 𝜆 in the slides, what is a correct value ?
  const auto query_point = tier4_autoware_utils::Point2d(x, y);
  const auto dist = boost::geometry::distance(query_point, centerline);
  return std::exp((dist * dist) / (2.0 * lambda_square)) - 1;
}

double get_simplified_road_bounds_pf(
  const double x, const double y, const tier4_autoware_utils::LineString2d & left_bound,
  const tier4_autoware_utils::LineString2d & right_bound)
{
  const auto query_point = tier4_autoware_utils::Point2d(x, y);
  constexpr auto cost_factor = 1.00;  // TODO(Lin): 𝜍 in the slides, what is a correct value ?
  const auto dist_left = boost::geometry::distance(query_point, left_bound);
  const auto dist_right = boost::geometry::distance(query_point, right_bound);
  const auto dist = std::max(std::min(dist_left, dist_right), 0.1);  // prevent division by 0.0
  return 0.5 * cost_factor * (1.0 / dist) * (1.0 / dist);
}
}  // namespace potential_fields
#endif  // POTENTIAL_FIELDS__POTENTIAL_FIELDS_HPP_
