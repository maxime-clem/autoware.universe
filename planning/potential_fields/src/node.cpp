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

#include "potential_fields/node.hpp"

#include "potential_fields/apf_curve_road.hpp"
#include "potential_fields/potential_fields.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace potential_fields
{

PotentialFieldsNode::PotentialFieldsNode(const rclcpp::NodeOptions & node_options)
: Node("potential_fields_node", node_options)
{
  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  // Publishers
  pub_grid_map_ =
    this->create_publisher<grid_map_msgs::msg::GridMap>("~/output/grid_map", rclcpp::QoS{1});

  // Subscribers
  sub_path_ = this->create_subscription<autoware_auto_planning_msgs::msg::Path>(
    "~/input/path", 1, std::bind(&PotentialFieldsNode::onPath, this, std::placeholders::_1));
  sub_dynamic_objects_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "~/input/objects", 1,
      std::bind(&PotentialFieldsNode::onDynamicObjects, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1,
    std::bind(&PotentialFieldsNode::onOdometry, this, std::placeholders::_1));
}

void PotentialFieldsNode::onPath(const autoware_auto_planning_msgs::msg::Path::ConstSharedPtr msg)
{
  path_ptr_ = msg;

  if (!odometry_ptr_) {
    RCLCPP_ERROR(get_logger(), "Odometry not received");
    return;
  }
  if (!objects_ptr_) {
    RCLCPP_ERROR(get_logger(), "Objects not received");
    return;
  }

  Info ego_info;
  ego_info.x = odometry_ptr_->pose.pose.position.x;
  ego_info.y = odometry_ptr_->pose.pose.position.y;
  // TODO(Maxime): calculate these correct vel/accel
  ego_info.v_lon = odometry_ptr_->twist.twist.linear.x;
  ego_info.v_lat = 0.0;
  ego_info.a_lat = 0.0;
  ego_info.a_lon = 0.0;
  ego_info.yaw = tf2::getYaw(odometry_ptr_->pose.pose.orientation);
  ego_info.m = 0.0;  // TODO(Lin): what is that ?
  ego_info.length = vehicle_info_.vehicle_length_m;
  ego_info.width = vehicle_info_.vehicle_width_m;

  grid_map::GridMap grid_map;
  grid_map.setFrameId("map");
  grid_map.setGeometry(
    grid_map::Length(50.0, 50.0), 1.0, grid_map::Position(ego_info.x, ego_info.y));
  grid_map.add("road_potential_field", 0.0);
  grid_map.add("obstacle_potential_field", 0.0);
  grid_map.add("sum", 0.0);

  Info obs_info;
  // TODO(Maxime)

  for (auto it = grid_map::GridMapIterator(grid_map); !it.isPastEnd(); ++it) {
    grid_map::Position pos;
    grid_map.getPosition(*it, pos);
    // grid_map.at("road_pf", *it) = getRoadPF(pos.x, pos.y);
    if (objects_ptr_ && !objects_ptr_->objects.empty()) {
      // TODO(Maxime): get closest obstacle
      // grid_map.at("road_pf", *it) = getObsPF(pos.x, pos.y, obs.x, obs.y);
    }
  }

  grid_map.add("pfs", grid_map["road_potential_field"] + grid_map["obstacle_potential_field"]);
  grid_map_msgs::msg::GridMap map_msg;
  map_msg = *grid_map::GridMapRosConverter::toMessage(grid_map);
  pub_grid_map_->publish(map_msg);
}

void PotentialFieldsNode::onDynamicObjects(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  objects_ptr_ = msg;
}

void PotentialFieldsNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odometry_ptr_ = msg;
}
}  // namespace potential_fields

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(potential_fields::PotentialFieldsNode)
