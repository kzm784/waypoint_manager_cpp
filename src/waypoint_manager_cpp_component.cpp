#include "waypoint_manager_cpp/waypoint_manager_cpp_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace waypoint_manager_cpp
{
WaypointManagerCppComponent::WaypointManagerCppComponent(const rclcpp::NodeOptions & options)
: Node("waypoint_manager_cpp", options)
{
    // Parameters
    declare_parameter<std::string>("waypoints_csv", "");
    declare_parameter<std::int32_t>("start_waypoint_id", 0);
    declare_parameter<bool>("loop_enable", false);
    declare_parameter<std::int32_t>("loop_count", 0);
    get_parameter("waypoints_csv", waypoints_csv_);
    get_parameter("start_waypoint_id", start_waypoint_id_);
    get_parameter("loop_enable", loop_enable_);
    get_parameter("loop_count", loop_count_);
    RCLCPP_INFO(get_logger(), "waypoints_csv: %s", waypoints_csv_.c_str());
    RCLCPP_INFO(get_logger(), "start_waypoint_id: %d", start_waypoint_id_);
    RCLCPP_INFO(get_logger(), "loop_enable: %s", loop_enable_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "loop_count: %d", loop_count_);
    

    RCLCPP_INFO(this->get_logger(), "Start waypoint_manager_cpp!");
}
}  // namespace waypoint_manager_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_manager_cpp::WaypointManagerCppComponent)