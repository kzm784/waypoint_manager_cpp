#include "waypoint_manager_cpp/waypoint_manager_cpp_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace waypoint_manager_cpp
{

WaypointManagerCppComponent::WaypointManagerCppComponent(const rclcpp::NodeOptions & options)
: Node("waypoint_manager_cpp", options), waypoint_id_(0), skip_enable_(false), event_enable_(false)
{
    // Parameters
    declare_parameter<std::string>("waypoints_csv", "");
    declare_parameter<std::int32_t>("start_waypoint_id", 0);
    declare_parameter<bool>("loop_enable", false);
    declare_parameter<std::int32_t>("loop_count", 0);

    get_parameter("waypoints_csv", waypoints_csv_);
    get_parameter("start_waypoint_id", waypoint_id_);
    get_parameter("loop_enable", loop_enable_);
    get_parameter("loop_count", loop_count_);

    RCLCPP_INFO(get_logger(), "waypoints_csv: %s", waypoints_csv_.c_str());
    RCLCPP_INFO(get_logger(), "start_waypoint_id: %d", waypoint_id_);
    RCLCPP_INFO(get_logger(), "loop_enable: %s", loop_enable_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "loop_count: %d", loop_count_);

    // Subscribers
    skip_flag_sub_ = create_subscription<std_msgs::msg::Bool>(
        "skip_flag", 1, std::bind(&WaypointManagerCppComponent::skipFlagCallback, this, std::placeholders::_1));
    event_flag_sub_ = create_subscription<std_msgs::msg::Int32>(
        "event_flag", 1, std::bind(&WaypointManagerCppComponent::eventFlagCallback, this, std::placeholders::_1));

    // Publishers
    next_waypoint_id_pub_ = create_publisher<std_msgs::msg::Int32>("next_waypoint_id", 1);
    reached_waypoint_id_pub_ = create_publisher<std_msgs::msg::Int32>("reached_waypoint_id", 1);

    // Action client for navigation
    nav2_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    // Load Waypoints from CSV
    LoadWaypoints loader;
    waypoints_data_ = loader.loadWaypointsFromCSV(waypoints_csv_);
    if (waypoints_data_.empty())
    {
        RCLCPP_ERROR(get_logger(), "No waypoints loaded. Please check the CSV file.");
        return;
    }

    // Start waypoint navigation
    updateGoal();
}

void WaypointManagerCppComponent::updateGoal()
{
    if (waypoints_data_.empty() || waypoint_id_ >= static_cast<int>(waypoints_data_.size())) return;

    RCLCPP_INFO(get_logger(), "Update waypoint ID: %d", waypoint_id_);

    // Create PoseStamped for goal
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = get_clock()->now();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = std::stof(waypoints_data_[waypoint_id_][1]);
    goal_pose.pose.position.y = std::stof(waypoints_data_[waypoint_id_][2]);
    goal_pose.pose.position.z = std::stof(waypoints_data_[waypoint_id_][3]);
    goal_pose.pose.orientation.x = std::stof(waypoints_data_[waypoint_id_][4]);
    goal_pose.pose.orientation.y = std::stof(waypoints_data_[waypoint_id_][5]);
    goal_pose.pose.orientation.z = std::stof(waypoints_data_[waypoint_id_][6]);
    goal_pose.pose.orientation.w = std::stof(waypoints_data_[waypoint_id_][7]);

    // Publish next waypoint ID
    std_msgs::msg::Int32 next_waypoint_msg;
    next_waypoint_msg.data = waypoint_id_;
    next_waypoint_id_pub_->publish(next_waypoint_msg);

    // Send goal to Nav2
    sendGoal(goal_pose);
}

void WaypointManagerCppComponent::updateWaypoint()
{
    waypoint_id_++;
    if (waypoint_id_ >= static_cast<int>(waypoints_data_.size()))
    {
        loop_count_--;
        if (!loop_enable_ || (loop_count_ < 1))
        {
            RCLCPP_INFO(get_logger(), "Completed Navigation!");
            return;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Completed a lap, %d laps left.", loop_count_);
            waypoint_id_ = 0;
            return;
        }
    }
}

void WaypointManagerCppComponent::sendGoal(const geometry_msgs::msg::PoseStamped& goal_pose)
{
    if (!nav2_pose_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_WARN(get_logger(), "Navigation server is not available, waiting...");
        return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        [this](const auto& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                updateWaypoint();
                updateGoal();
                return;
            }
            else if (result.code == rclcpp_action::ResultCode::ABORTED)
            {
                return;
            }
            else if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                return;
            }
            else
            {
                return;
            }
        };

    nav2_pose_client_->async_send_goal(goal_msg, send_goal_options);
}

void WaypointManagerCppComponent::skipFlagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data || !skip_enable_) return;

    RCLCPP_INFO(get_logger(), "Skip flag received. Moving to the next waypoint...");
    waypoint_id_++;
    updateGoal();
}

void WaypointManagerCppComponent::eventFlagCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (!event_enable_) return;

    RCLCPP_INFO(get_logger(), "Event flag received: %d", msg->data);
}

}  // namespace waypoint_manager_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_manager_cpp::WaypointManagerCppComponent)
