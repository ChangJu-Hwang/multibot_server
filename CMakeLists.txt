################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(multibot_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(multibot_ros2_interface REQUIRED)
find_package(multibot_util REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(gazebo_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

################################################################################
# Build
################################################################################
include_directories(
  include
)

# server
set(SERVER_DEPENDENCIES
  "rclcpp"
  "nav_msgs"
  "std_msgs"
  "visualization_msgs"
  "multibot_ros2_interface"
  "multibot_util"
  "Qt5"
)

set(PANEL_SOURCES "include/multibot_server/server_panel.hpp")

set(SERVER_EXEC_NAME "server")
file(GLOB SERVER_SOURCES "src/*.cpp")
add_executable(${SERVER_EXEC_NAME} ${SERVER_SOURCES} ${PANEL_SOURCES})

target_link_libraries(${SERVER_EXEC_NAME} Qt5::Widgets)
ament_target_dependencies(${SERVER_EXEC_NAME} ${SERVER_DEPENDENCIES})

target_include_directories(${SERVER_EXEC_NAME} PUBLIC
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCUDE_DIRS})
target_link_libraries( ${SERVER_EXEC_NAME} ${YAML_CPP_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS
  ${SERVER_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  Instances/maps
  launch
  models
  rviz
  ui
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multibot_server.dsv.in")

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(Qt5)

################################################################################
# Build test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


################################################################################
# Find and load build settings from external packages
################################################################################
ament_package()