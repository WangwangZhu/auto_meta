cmake_minimum_required(VERSION 3.10)
project(lqr_pid_trajectory_tracking)

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
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(chassis_msg REQUIRED)
find_package(matrix_interfaces REQUIRED)
find_package(carla_msgs REQUIRED)
# message("herererererer")
find_package(Boost REQUIRED COMPONENTS system chrono regex thread program_options date_time)

find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME} "msg/VehicleControl.msg")

add_executable(${PROJECT_NAME}_node 
                    src/lqr_pid_trajectory_tracking.cpp 
                    src/lqr_pid_trajectory_tracking_controller.cpp
                    src/pid_controller.cpp
                    src/reference_line.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_node ipopt z ssl uv uWS)
ament_target_dependencies(${PROJECT_NAME}_node "std_msgs" "rclcpp" "chassis_msg" "matrix_interfaces" "nav_msgs" "visualization_msgs" "sensor_msgs" "tf2" "tf2_ros" "tf2_geometry_msgs" "carla_msgs")# ipopt z ssl uv uWS


install(TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
