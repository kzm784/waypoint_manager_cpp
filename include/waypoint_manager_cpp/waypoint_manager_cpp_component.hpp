#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "waypoint_manager_cpp/visibility_control.h"

namespace waypoint_manager_cpp
{
class WaypointManagerCppComponent : public rclcpp::Node
{
public:
  WAYPOINT_MANAGER_CPP_PUBLIC
  explicit WaypointManagerCppComponent(const rclcpp::NodeOptions & options);

private:
  void SkipFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void EventFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr skip_flag_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr event_flag_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::ConstSharedPtr next_waypoint_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::ConstSharedPtr reached_waypoint_id_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::ConstSharedPtr nav2_pose_client_;

  // Parameters
  std::string waypoints_csv_;
  bool loop_enable_{false};
  std::int32_t loop_count_;
  std::int32_t start_waypoint_id_;
};
}  // namespace waypoint_manager_cpp