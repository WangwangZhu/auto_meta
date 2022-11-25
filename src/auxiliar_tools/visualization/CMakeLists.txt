cmake_minimum_required(VERSION 3.10)
project(visualization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# message("find visual")
add_executable(host_vehicle_visualization src/host_vehicle_visualization.cpp)
target_include_directories(host_vehicle_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(host_vehicle_visualization
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  visualization_msgs
  tf2_geometry_msgs
  )
install(TARGETS host_vehicle_visualization
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

add_executable(global_path_visualization src/global_path_visualization.cpp)
target_include_directories(global_path_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(global_path_visualization
    rclcpp
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
    visualization_msgs
    tf2_geometry_msgs
    )
install(TARGETS global_path_visualization
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
