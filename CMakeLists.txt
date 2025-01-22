cmake_minimum_required(VERSION 3.8)
project(waypoint_manager_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/waypoint_manager_cpp_component.cpp
)
target_compile_definitions(${PROJECT_NAME}_component
  PRIVATE "${PROJECT_NAME}_BUILDING_DLL"
)
rclcpp_components_register_nodes(${PROJECT_NAME}_component
  "${PROJECT_NAME}::WaypointManagerCppComponent"
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/waypoint_manager_cpp_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_component
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
