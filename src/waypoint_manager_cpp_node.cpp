#include <memory>
#include <waypoint_manager_cpp/waypoint_manager_cpp_component.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<waypoint_manager_cpp::WaypointManagerCppComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}