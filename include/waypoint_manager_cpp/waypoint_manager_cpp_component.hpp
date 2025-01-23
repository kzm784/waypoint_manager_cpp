#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "waypoint_manager_cpp/visibility_control.h"

namespace waypoint_manager_cpp
{
class WaypointManagerCppComponent : public rclcpp::Node
{
public:
  WAYPOINT_MANAGER_CPP_PUBLIC
  explicit WaypointManagerCppComponent(const rclcpp::NodeOptions & options);

private:
  // Parameters
  std::string waypoints_csv_;
  bool loop_enable_{false};
  std::int32_t loop_count_;
  std::int32_t start_waypoint_id_;
};
}  // namespace waypoint_manager_cpp