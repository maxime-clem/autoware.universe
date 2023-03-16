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

#ifndef POTENTIAL_FIELDS__NODE_HPP_
#define POTENTIAL_FIELDS__NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>

namespace potential_fields
{

class PotentialFieldsNode : public rclcpp::Node
{
public:
  explicit PotentialFieldsNode(const rclcpp::NodeOptions & node_options);

private:
  void onPath(const autoware_auto_planning_msgs::msg::Path::ConstSharedPtr msg);
  void onDynamicObjects(
    const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // cached data
  vehicle_info_util::VehicleInfo vehicle_info_;
  autoware_auto_planning_msgs::msg::Path::ConstSharedPtr path_ptr_;
  autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr_;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_ptr_;
  // publisher and subscriber
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    sub_dynamic_objects_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_;
};
}  // namespace potential_fields

#endif  // POTENTIAL_FIELDS__NODE_HPP_
