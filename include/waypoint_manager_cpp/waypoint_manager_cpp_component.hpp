#pragma once

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "waypoint_manager_cpp/load_waypoints.hpp"

namespace waypoint_manager_cpp
{
class WaypointManagerCppComponent : public rclcpp::Node
{
public:
    explicit WaypointManagerCppComponent(const rclcpp::NodeOptions & options);

private:
    void skipFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void eventFlagCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose);
    void updateWaypoint();
    void updateGoal();

    // Parameters
    std::string waypoints_csv_;
    bool loop_enable_{false};
    int loop_count_{0};
    int waypoint_id_{0};

    // State variables
    bool retry_once_{false};
    bool skip_enable_{false};
    bool event_enable_{false};
    bool nav2_enable_{true};
    std::vector<std::vector<std::string>> waypoints_data_;
    geometry_msgs::msg::PoseStamped pose_msg_;

    // ROS 2 Interfaces
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr skip_flag_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr event_flag_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr next_waypoint_id_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr reached_waypoint_id_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_pose_client_;
};
}  // namespace waypoint_manager_cpp
