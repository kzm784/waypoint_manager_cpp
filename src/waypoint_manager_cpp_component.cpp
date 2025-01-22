#include "waypoint_manager_cpp/waypoint_manager_cpp_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace waypoint_manager_cpp
{
WaypointManagerCppComponent::WaypointManagerCppComponent(const rclcpp::NodeOptions & options)
: Node("waypoint_manager_cpp", options)
{
    RCLCPP_INFO(this->get_logger(), "Start waypoint_manager_cpp!");
}
}  // namespace waypoint_manager_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_manager_cpp::WaypointManagerCppComponent)