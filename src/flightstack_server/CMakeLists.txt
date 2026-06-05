cmake_minimum_required(VERSION 3.8)
project(flightstack_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeControl.srv"
  DEPENDENCIES px4_msgs
)

# Create executable
add_executable(flightstack_server_dummy src/flightstack_server_dummy.cpp)

ament_target_dependencies(flightstack_server_dummy
  rclcpp
  px4_msgs
)

# Link generated service
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(flightstack_server_dummy "${cpp_typesupport_target}")

# Install
install(TARGETS
  flightstack_server_dummy
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
