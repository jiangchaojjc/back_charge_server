cmake_minimum_required(VERSION 3.5)
project(back_charge_server)

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

find_package(charge_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcsbot_interface REQUIRED)

find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(charge_action_server SHARED
  src/back_charge_server.cpp)

target_include_directories(charge_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(charge_action_server
  PRIVATE "CHARGE_ACTION_BUILDING_DLL")

ament_target_dependencies(charge_action_server
  "charge_interface"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "rcsbot_interface"
  "geometry_msgs")

rclcpp_components_register_node(charge_action_server PLUGIN "action_ChargeBack::ChargeBack" EXECUTABLE back_charge_server)

install(TARGETS
charge_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
