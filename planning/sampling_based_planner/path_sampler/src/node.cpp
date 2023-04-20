// Copyright 2023 TIER IV, Inc.
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

#include "path_sampler/node.hpp"

#include "interpolation/spline_interpolation_points_2d.hpp"
#include "path_sampler/path_generation.hpp"
#include "path_sampler/prepare_inputs.hpp"
#include "path_sampler/utils/geometry_utils.hpp"
#include "path_sampler/utils/trajectory_utils.hpp"
#include "rclcpp/time.hpp"
#include "sampler_common/constraints/hard_constraint.hpp"
#include "sampler_common/constraints/soft_constraint.hpp"

#include <chrono>
#include <limits>

namespace path_sampler
{
namespace
{
template <class T>
std::vector<T> concatVectors(const std::vector<T> & prev_vector, const std::vector<T> & next_vector)
{
  std::vector<T> concatenated_vector;
  concatenated_vector.insert(concatenated_vector.end(), prev_vector.begin(), prev_vector.end());
  concatenated_vector.insert(concatenated_vector.end(), next_vector.begin(), next_vector.end());
  return concatenated_vector;
}

StringStamped createStringStamped(const rclcpp::Time & now, const std::string & data)
{
  StringStamped msg;
  msg.stamp = now;
  msg.data = data;
  return msg;
}

bool hasZeroVelocity(const TrajectoryPoint & traj_point)
{
  constexpr double zero_vel = 0.0001;
  return std::abs(traj_point.longitudinal_velocity_mps) < zero_vel;
}
}  // namespace

PathSampler::PathSampler(const rclcpp::NodeOptions & node_options)
: Node("path_sampler", node_options),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo()),
  time_keeper_ptr_(std::make_shared<TimeKeeper>())
{
  // interface publisher
  traj_pub_ = create_publisher<Trajectory>("~/output/path", 1);
  virtual_wall_pub_ = create_publisher<MarkerArray>("~/virtual_wall", 1);

  // interface subscriber
  path_sub_ = create_subscription<Path>(
    "~/input/path", 1, std::bind(&PathSampler::onPath, this, std::placeholders::_1));
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odometry", 1, [this](const Odometry::SharedPtr msg) { ego_state_ptr_ = msg; });
  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", 1, std::bind(&PathSampler::objectsCallback, this, std::placeholders::_1));

  // debug publisher
  debug_markers_pub_ = create_publisher<MarkerArray>("~/debug/marker", 1);
  debug_calculation_time_pub_ = create_publisher<StringStamped>("~/debug/calculation_time", 1);

  {  // parameters
    params_.constraints.hard.max_curvature =
      declare_parameter<double>("constraints.hard.max_curvature");
    params_.constraints.hard.min_curvature =
      declare_parameter<double>("constraints.hard.min_curvature");
    params_.constraints.soft.lateral_deviation_weight =
      declare_parameter<double>("constraints.soft.lateral_deviation_weight");
    params_.constraints.soft.longitudinal_deviation_weight =
      declare_parameter<double>("constraints.soft.longitudinal_deviation_weight");
    params_.constraints.soft.jerk_weight =
      declare_parameter<double>("constraints.soft.jerk_weight");
    params_.constraints.soft.length_weight =
      declare_parameter<double>("constraints.soft.length_weight");
    params_.constraints.soft.curvature_weight =
      declare_parameter<double>("constraints.soft.curvature_weight");
    params_.sampling.enable_frenet = declare_parameter<bool>("sampling.enable_frenet");
    params_.sampling.enable_bezier = declare_parameter<bool>("sampling.enable_bezier");
    params_.sampling.resolution = declare_parameter<double>("sampling.resolution");
    params_.sampling.minimum_committed_length =
      declare_parameter<double>("sampling.minimum_committed_length");
    params_.sampling.target_lengths =
      declare_parameter<std::vector<double>>("sampling.target_lengths");
    params_.sampling.frenet.target_lateral_positions =
      declare_parameter<std::vector<double>>("sampling.frenet.target_lateral_positions");
    params_.sampling.frenet.target_lateral_velocities =
      declare_parameter<std::vector<double>>("sampling.frenet.target_lateral_velocities");
    params_.sampling.frenet.target_lateral_accelerations =
      declare_parameter<std::vector<double>>("sampling.frenet.target_lateral_accelerations");
    params_.sampling.bezier.nb_k = declare_parameter<int>("sampling.bezier.nb_k");
    params_.sampling.bezier.mk_min = declare_parameter<double>("sampling.bezier.mk_min");
    params_.sampling.bezier.mk_max = declare_parameter<double>("sampling.bezier.mk_max");
    params_.sampling.bezier.nb_t = declare_parameter<int>("sampling.bezier.nb_t");
    params_.sampling.bezier.mt_min = declare_parameter<double>("sampling.bezier.mt_min");
    params_.sampling.bezier.mt_max = declare_parameter<double>("sampling.bezier.mt_max");
    params_.preprocessing.force_zero_deviation =
      declare_parameter<bool>("preprocessing.force_zero_initial_deviation");
    params_.preprocessing.force_zero_heading =
      declare_parameter<bool>("preprocessing.force_zero_initial_heading");
    params_.preprocessing.smooth_reference =
      declare_parameter<bool>("preprocessing.smooth_reference_trajectory");
    params_.constraints.ego_footprint = vehicle_info_.createFootprint();

    // parameter for debug info
    time_keeper_ptr_->enable_calculation_time_info =
      declare_parameter<bool>("debug.enable_calculation_time_info");

    // parameters for ego nearest search
    ego_nearest_param_ = EgoNearestParam(this);

    // parameters for trajectory
    traj_param_ = TrajectoryParam(this);
  }

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PathSampler::onParam, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult PathSampler::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  updateParam(parameters, "constraints.hard.max_curvature", params_.constraints.hard.max_curvature);
  updateParam(parameters, "constraints.hard.min_curvature", params_.constraints.hard.min_curvature);
  updateParam(
    parameters, "constraints.soft.lateral_deviation_weight",
    params_.constraints.soft.lateral_deviation_weight);
  updateParam(
    parameters, "constraints.soft.longitudinal_deviation_weight",
    params_.constraints.soft.longitudinal_deviation_weight);
  updateParam(parameters, "constraints.soft.jerk_weight", params_.constraints.soft.jerk_weight);
  updateParam(parameters, "constraints.soft.length_weight", params_.constraints.soft.length_weight);
  updateParam(
    parameters, "constraints.soft.curvature_weight", params_.constraints.soft.curvature_weight);
  updateParam(parameters, "sampling.enable_frenet", params_.sampling.enable_frenet);
  updateParam(parameters, "sampling.enable_bezier", params_.sampling.enable_bezier);
  updateParam(parameters, "sampling.resolution", params_.sampling.resolution);
  updateParam(
    parameters, "sampling.minimum_committed_length", params_.sampling.minimum_committed_length);
  updateParam(parameters, "sampling.target_lengths", params_.sampling.target_lengths);
  updateParam(
    parameters, "sampling.frenet.target_lateral_positions",
    params_.sampling.frenet.target_lateral_positions);
  updateParam(
    parameters, "sampling.frenet.target_lateral_velocities",
    params_.sampling.frenet.target_lateral_velocities);
  updateParam(
    parameters, "sampling.frenet.target_lateral_accelerations",
    params_.sampling.frenet.target_lateral_accelerations);
  updateParam(parameters, "sampling.bezier.nb_k", params_.sampling.bezier.nb_k);
  updateParam(parameters, "sampling.bezier.mk_min", params_.sampling.bezier.mk_min);
  updateParam(parameters, "sampling.bezier.mk_max", params_.sampling.bezier.mk_max);
  updateParam(parameters, "sampling.bezier.nb_t", params_.sampling.bezier.nb_t);
  updateParam(parameters, "sampling.bezier.mt_min", params_.sampling.bezier.mt_min);
  updateParam(parameters, "sampling.bezier.mt_max", params_.sampling.bezier.mt_max);
  updateParam(
    parameters, "preprocessing.force_zero_initial_deviation",
    params_.preprocessing.force_zero_deviation);
  updateParam(
    parameters, "preprocessing.force_zero_initial_heading",
    params_.preprocessing.force_zero_heading);
  updateParam(
    parameters, "preprocessing.smooth_reference_trajectory",
    params_.preprocessing.smooth_reference);
  updateParam(
    parameters, "debug.enable_calculation_time_info",
    time_keeper_ptr_->enable_calculation_time_info);
  // parameters for ego nearest search
  ego_nearest_param_.onParam(parameters);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void PathSampler::resetPreviousData()
{
  prev_path_.reset();
}

void PathSampler::objectsCallback(const PredictedObjects::SharedPtr msg)
{
  in_objects_ptr_ = msg;
}

sampler_common::State PathSampler::getPlanningState(
  sampler_common::State & state, const sampler_common::transform::Spline2D & path_spline) const
{
  state.frenet = path_spline.frenet(state.pose);
  if (params_.preprocessing.force_zero_deviation) {
    state.pose = path_spline.cartesian(state.frenet.s);
  }
  if (params_.preprocessing.force_zero_heading) {
    state.heading = path_spline.yaw(state.frenet.s);
  }
  // TODO(Maxime): curvature from steering angle ?
  state.curvature = path_spline.curvature(state.frenet.s);
  return state;
}

void PathSampler::onPath(const Path::SharedPtr path_ptr)
{
  time_keeper_ptr_->init();
  time_keeper_ptr_->tic(__func__);

  // check if data is ready and valid
  if (!isDataReady(*path_ptr, *get_clock())) {
    return;
  }

  // 0. return if path is backward
  // TODO(Maxime): support backward path
  if (!motion_utils::isDrivingForward(path_ptr->points)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Backward path is NOT supported. Just converting path to trajectory");
  }
  // 1. create planner data
  const auto planner_data = createPlannerData(*path_ptr);
  // 2. generate trajectory
  const auto generated_traj_points = generateTrajectory(planner_data);
  // 3. extend trajectory to connect the optimized trajectory and the following path smoothly
  if (!generated_traj_points.empty()) {
    auto full_traj_points = extendTrajectory(planner_data.traj_points, generated_traj_points);
    const auto output_traj_msg =
      trajectory_utils::createTrajectory(path_ptr->header, full_traj_points);
    traj_pub_->publish(output_traj_msg);
  }

  time_keeper_ptr_->toc(__func__, "");
  *time_keeper_ptr_ << "========================================";
  time_keeper_ptr_->endLine();

  // publish calculation_time
  // NOTE: This function must be called after measuring onPath calculation time
  const auto calculation_time_msg = createStringStamped(now(), time_keeper_ptr_->getLog());
  debug_calculation_time_pub_->publish(calculation_time_msg);
}

bool PathSampler::isDataReady(const Path & path, rclcpp::Clock clock) const
{
  if (!ego_state_ptr_) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), clock, 5000, "Waiting for ego pose and twist.");
    return false;
  }

  if (path.points.size() < 2) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), clock, 5000, "Path points size is less than 1.");
    return false;
  }

  if (path.left_bound.empty() || path.right_bound.empty()) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), clock, 5000, "Left or right bound in path is empty.");
    return false;
  }

  return true;
}

PlannerData PathSampler::createPlannerData(const Path & path) const
{
  // create planner data
  PlannerData planner_data;
  planner_data.header = path.header;
  planner_data.traj_points = trajectory_utils::convertToTrajectoryPoints(path.points);
  planner_data.left_bound = path.left_bound;
  planner_data.right_bound = path.right_bound;
  planner_data.ego_pose = ego_state_ptr_->pose.pose;
  planner_data.ego_vel = ego_state_ptr_->twist.twist.linear.x;
  return planner_data;
}

std::vector<TrajectoryPoint> PathSampler::generateTrajectory(const PlannerData & planner_data)
{
  time_keeper_ptr_->tic(__func__);

  const auto & input_traj_points = planner_data.traj_points;

  auto generated_traj_points = generatePath(planner_data);

  // 2. update velocity
  applyInputVelocity(generated_traj_points, input_traj_points, planner_data.ego_pose);

  // 4. publish debug marker
  publishDebugMarkerOfOptimization(generated_traj_points);

  time_keeper_ptr_->toc(__func__, " ");
  return generated_traj_points;
}

std::vector<TrajectoryPoint> PathSampler::generatePath(const PlannerData & planner_data)
{
  std::vector<TrajectoryPoint> trajectory;
  time_keeper_ptr_->tic(__func__);
  const auto & p = planner_data;

  const auto path_spline = preparePathSpline(p.traj_points, params_.preprocessing.smooth_reference);
  sampler_common::State current_state;
  current_state.pose = {planner_data.ego_pose.position.x, planner_data.ego_pose.position.y};
  current_state.heading = tf2::getYaw(planner_data.ego_pose.orientation);

  const auto planning_state = getPlanningState(current_state, path_spline);
  prepareConstraints(params_.constraints, *in_objects_ptr_, p.left_bound, p.right_bound);
  params_.constraints.distance_to_end = path_spline.lastS() - planning_state.frenet.s;

  auto candidate_paths = generateCandidatePaths(planning_state, path_spline, params_);
  /* TODO(Maxime): reimplement reusing the previous path
  const auto updated_prev_path = updatePreviousPath(
    prev_path_, planning_state, params_.preprocessing.reuse_max_deviation);
  auto reusable_paths = sampler_common::calculateReusablePaths(
    updated_prev_path, params_.preprocessing.reuse_lengths);
  for (auto & reusable_path: reusable_paths) {
    auto paths = generateCandidatePaths(
      reusable_path.planning_state, reusable_path.path, path_spline, *path_ptr, params_);
    candidate_paths.insert(
      candidate_paths.end(), std::make_move_iterator(paths.begin()),
      std::make_move_iterator(paths.end()));
  }
  */
  debug_data_.footprints.clear();
  for (auto & path : candidate_paths) {
    // TODO(Maxime): resample the path ?
    const auto footprint =
      sampler_common::constraints::checkHardConstraints(path, params_.constraints);
    debug_data_.footprints.push_back(footprint);
    sampler_common::constraints::calculateCost(path, params_.constraints, path_spline);
  }
  const auto best_path_idx = [](const auto & paths) {
    auto min_cost = std::numeric_limits<double>::max();
    size_t best_path_idx = 0;
    for (auto i = 0LU; i < paths.size(); ++i) {
      if (paths[i].constraint_results.isValid() && paths[i].cost < min_cost) {
        best_path_idx = i;
        min_cost = paths[i].cost;
      }
    }
    return min_cost < std::numeric_limits<double>::max() ? std::optional<size_t>(best_path_idx)
                                                         : std::nullopt;
  };
  const auto selected_path_idx = best_path_idx(candidate_paths);
  if (selected_path_idx) {
    const auto & selected_path = candidate_paths[*selected_path_idx];
    trajectory = trajectory_utils::convertToTrajectoryPoints(selected_path);
    prev_path_ = std::make_shared<std::vector<TrajectoryPoint>>(trajectory);
  } else {
    std::printf("No valid path found (out of %lu) outputing input path \n", candidate_paths.size());
    int coll = 0;
    int da = 0;
    int k = 0;
    for (const auto & p : candidate_paths) {
      coll += static_cast<int>(!p.constraint_results.collision);
      da += static_cast<int>(!p.constraint_results.drivable_area);
      k += static_cast<int>(!p.constraint_results.curvature);
    }
    std::printf("\tInvalid coll/da/k = %d/%d/%d\n", coll, da, k);
    trajectory = trajectory_utils::convertToTrajectoryPoints(p.traj_points);
  }
  time_keeper_ptr_->toc(__func__, "    ");
  debug_data_.previous_sampled_candidates_nb = debug_data_.sampled_candidates.size();
  debug_data_.sampled_candidates = candidate_paths;
  debug_data_.obstacles = params_.constraints.obstacle_polygons;
  return trajectory;
}

std::vector<TrajectoryPoint> PathSampler::getPrevOptimizedTrajectory(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  if (prev_path_) {
    return *prev_path_;
  }

  return traj_points;
}

void PathSampler::applyInputVelocity(
  std::vector<TrajectoryPoint> & output_traj_points,
  const std::vector<TrajectoryPoint> & input_traj_points,
  const geometry_msgs::msg::Pose & ego_pose) const
{
  if (output_traj_points.empty()) return;
  time_keeper_ptr_->tic(__func__);

  // crop forward for faster calculation
  const auto forward_cropped_input_traj_points = [&]() {
    const double generated_traj_length = motion_utils::calcArcLength(output_traj_points);
    constexpr double margin_traj_length = 10.0;

    const size_t ego_seg_idx =
      trajectory_utils::findEgoSegmentIndex(input_traj_points, ego_pose, ego_nearest_param_);
    return motion_utils::cropForwardPoints(
      input_traj_points, ego_pose.position, ego_seg_idx,
      generated_traj_length + margin_traj_length);
  }();

  // update velocity
  size_t input_traj_start_idx = 0;
  for (size_t i = 0; i < output_traj_points.size(); i++) {
    // crop backward for efficient calculation
    const auto cropped_input_traj_points = std::vector<TrajectoryPoint>{
      forward_cropped_input_traj_points.begin() + input_traj_start_idx,
      forward_cropped_input_traj_points.end()};

    const size_t nearest_seg_idx = trajectory_utils::findEgoSegmentIndex(
      cropped_input_traj_points, output_traj_points.at(i).pose, ego_nearest_param_);
    input_traj_start_idx = nearest_seg_idx;

    // calculate velocity with zero order hold
    const double velocity = cropped_input_traj_points.at(nearest_seg_idx).longitudinal_velocity_mps;
    output_traj_points.at(i).longitudinal_velocity_mps = velocity;
  }

  // insert stop point explicitly
  const auto stop_idx = motion_utils::searchZeroVelocityIndex(forward_cropped_input_traj_points);
  if (stop_idx) {
    const auto input_stop_pose = forward_cropped_input_traj_points.at(stop_idx.get()).pose;
    const size_t stop_seg_idx = trajectory_utils::findEgoSegmentIndex(
      output_traj_points, input_stop_pose, ego_nearest_param_);

    // calculate and insert stop pose on output trajectory
    trajectory_utils::insertStopPoint(output_traj_points, input_stop_pose, stop_seg_idx);
  }

  time_keeper_ptr_->toc(__func__, "    ");
}

void PathSampler::publishVirtualWall(const geometry_msgs::msg::Pose & stop_pose) const
{
  time_keeper_ptr_->tic(__func__);

  const auto virtual_wall_marker = motion_utils::createStopVirtualWallMarker(
    stop_pose, "outside drivable area", now(), 0, vehicle_info_.max_longitudinal_offset_m);

  virtual_wall_pub_->publish(virtual_wall_marker);
  time_keeper_ptr_->toc(__func__, "      ");
}

void PathSampler::publishDebugMarkerOfOptimization(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  (void)traj_points;

  time_keeper_ptr_->tic(__func__);

  // debug marker
  time_keeper_ptr_->tic("getDebugMarker");
  // const auto markers = getDebugMarker(debug_data_);
  visualization_msgs::msg::MarkerArray markers;
  if (debug_markers_pub_->get_subscription_count() > 0LU) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.action = m.ADD;
    m.id = 0UL;
    m.type = m.LINE_STRIP;
    m.color.a = 1.0;
    m.scale.x = 0.02;
    m.ns = "candidates";
    for (const auto & c : debug_data_.sampled_candidates) {
      m.points.clear();
      for (const auto & p : c.points)
        m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
      if (c.constraint_results.isValid()) {
        m.color.g = 1.0;
        m.color.r = 0.0;
      } else {
        m.color.r = 1.0;
        m.color.g = 0.0;
      }
      markers.markers.push_back(m);
      ++m.id;
    }
    if (debug_data_.footprints.size() == 1UL) {
      m.ns = "footprints";
      m.id = 0UL;
      m.type = m.POINTS;
      m.points.clear();
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.scale.y = 0.02;
      for (const auto & p : debug_data_.footprints.front())
        m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
      markers.markers.push_back(m);
    }
    m.type = m.LINE_STRIP;
    m.ns = "obstacles";
    m.id = 0UL;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    for (const auto & obs : debug_data_.obstacles) {
      m.points.clear();
      for (const auto & p : obs.outer())
        m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
      markers.markers.push_back(m);
      ++m.id;
    }
    m.action = m.DELETE;
    m.ns = "candidates";
    for (m.id = debug_data_.sampled_candidates.size();
         static_cast<size_t>(m.id) < debug_data_.previous_sampled_candidates_nb; ++m.id)
      markers.markers.push_back(m);
  }
  time_keeper_ptr_->toc("getDebugMarker", "      ");

  time_keeper_ptr_->tic("publishDebugMarker");
  debug_markers_pub_->publish(markers);
  time_keeper_ptr_->toc("publishDebugMarker", "      ");

  time_keeper_ptr_->toc(__func__, "    ");
}

std::vector<TrajectoryPoint> PathSampler::extendTrajectory(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & optimized_traj_points) const
{
  time_keeper_ptr_->tic(__func__);

  const auto & joint_start_pose = optimized_traj_points.front().pose;
  const auto & joint_end_pose = optimized_traj_points.back().pose;

  // calculate start idx of optimized points on path points
  const size_t joint_start_traj_seg_idx =
    trajectory_utils::findEgoSegmentIndex(traj_points, joint_start_pose, ego_nearest_param_);

  // crop trajectory for extension
  constexpr double joint_traj_length_for_smoothing = 5.0;
  const auto joint_end_upto_traj_point_idx = trajectory_utils::getPointIndexAfter(
    traj_points, joint_start_pose.position, joint_start_traj_seg_idx,
    joint_traj_length_for_smoothing);

  // calculate prepended trajectory points
  const auto prepended_traj_points = [&]() {
    if (joint_start_traj_seg_idx == 0UL) return traj_points;
    const auto pre_traj = std::vector<TrajectoryPoint>(
      traj_points.begin(), traj_points.begin() + joint_start_traj_seg_idx);
    return concatVectors(pre_traj, optimized_traj_points);
  }();

  // calculate end idx of optimized points on path points
  const size_t joint_end_traj_seg_idx =
    trajectory_utils::findEgoSegmentIndex(traj_points, joint_end_pose, ego_nearest_param_);

  // calculate full trajectory points
  const auto full_traj_points = [&]() {
    if (!joint_end_upto_traj_point_idx) {
      return prepended_traj_points;
    }
    const auto extended_traj_points = std::vector<TrajectoryPoint>{
      traj_points.begin() + *joint_end_upto_traj_point_idx, traj_points.end()};
    return concatVectors(prepended_traj_points, extended_traj_points);
  }();

  // resample trajectory points
  auto resampled_traj_points = trajectory_utils::resampleTrajectoryPoints(
    full_traj_points, traj_param_.output_delta_arc_length);

  // update velocity on joint
  for (size_t i = joint_end_traj_seg_idx + 1; i <= joint_end_upto_traj_point_idx; ++i) {
    if (hasZeroVelocity(traj_points.at(i))) {
      if (i != 0 && !hasZeroVelocity(traj_points.at(i - 1))) {
        // Here is when current point is 0 velocity, but previous point is not 0 velocity.
        const auto & input_stop_pose = traj_points.at(i).pose;
        const size_t stop_seg_idx = trajectory_utils::findEgoSegmentIndex(
          resampled_traj_points, input_stop_pose, ego_nearest_param_);

        // calculate and insert stop pose on output trajectory
        trajectory_utils::insertStopPoint(resampled_traj_points, input_stop_pose, stop_seg_idx);
      }
    }
  }

  // debug_data_ptr_->extended_traj_points =
  //   extended_traj_points ? *extended_traj_points : std::vector<TrajectoryPoint>();
  time_keeper_ptr_->toc(__func__, "  ");
  return resampled_traj_points;
}
}  // namespace path_sampler

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_sampler::PathSampler)
