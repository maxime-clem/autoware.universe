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

#include "sampler_common/constraints/hard_constraint.hpp"

#include "sampler_common/constraints/footprint.hpp"
#include "sampler_common/structures.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/detail/intersects/interface.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/core/cs.hpp>

#include <algorithm>
#include <csignal>
#include <iostream>

namespace sampler_common::constraints
{
bool satisfyMinMax(const std::vector<double> & values, const double min, const double max)
{
  for (const auto value : values) {
    if (value < min || value > max) return false;
  }
  return true;
}

bool belowCollisionDistance(
  const MultiPoint2d & footprint, const MultiPolygon2d & polygons, const double min_dist)
{
  for (const auto & polygon : polygons) {
    if (boost::geometry::distance(footprint, polygon) < min_dist) return true;
  }
  return false;
}

MultiPoint2d checkHardConstraints(Path & path, const Constraints & constraints)
{
  const auto footprint = buildFootprintPoints(path, constraints);
  if (!footprint.empty()) {
    if (belowCollisionDistance(
          footprint, constraints.obstacle_polygons, constraints.collision_distance_buffer)) {
      path.constraint_results.collision = false;
    }
    if (!boost::geometry::within(footprint, constraints.drivable_polygons)) {
      path.constraint_results.drivable_area = false;
    }
  }
  if (!satisfyMinMax(
        path.curvatures, constraints.hard.min_curvature, constraints.hard.max_curvature)) {
    path.constraint_results.curvature = false;
  }
  return footprint;
}
}  // namespace sampler_common::constraints
