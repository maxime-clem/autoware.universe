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

#include "potential_fields/potential_fields.hpp"

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
  sub_dynamic_objects_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "~/input/objects", 1,
      std::bind(&PotentialFieldsNode::onDynamicObjects, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1,
    std::bind(&PotentialFieldsNode::onOdometry, this, std::placeholders::_1));
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
