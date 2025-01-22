#pragma once

#include "waypoint_manager_cpp/visibility_control.h"

#include <rclcpp/rclcpp.hpp>

namespace waypoint_manager_cpp
{
class WaypointManagerCppComponent : public rclcpp::Node
{
public:
  WAYPOINT_MANAGER_CPP_PUBLIC
  explicit WaypointManagerCppComponent(const rclcpp::NodeOptions & options);

private:

};
}  // namespace waypoint_manager_cpp