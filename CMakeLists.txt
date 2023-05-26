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

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(multibot_ros2_interface REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(gazebo_ros REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

# server
set(SERVER_DEPENDENCIES
  "rclcpp"
  "multibot_ros2_interface"
)
set(SERVER_EXEC_NAME "server")
file(GLOB SERVER_SOURCES "src/server/*.cpp")
add_executable(${SERVER_EXEC_NAME} ${SERVER_SOURCES})
ament_target_dependencies(${SERVER_EXEC_NAME} ${SERVER_DEPENDENCIES})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCUDE_DIRS})
target_link_libraries( ${SERVER_EXEC_NAME} ${YAML_CPP_LIBRARIES})

# simulation
set(SIMUL_DEPENDENCIES
  "rclcpp"
  "tf2"
  "visualization_msgs"
  "geometry_msgs"
  "nav_msgs"
  "multibot_ros2_interface"
)
set(SIMUL_EXEC_NAME "simulation")
file(GLOB SIMUL_SOURCES "src/simulation/*.cpp")
add_executable(${SIMUL_EXEC_NAME} ${SIMUL_SOURCES})
ament_target_dependencies(${SIMUL_EXEC_NAME} ${SIMUL_DEPENDENCIES})

################################################################################
# Install
################################################################################
install(TARGETS
  ${SERVER_EXEC_NAME}
  ${SIMUL_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  Instances/agents
  Instances/maps
  Instances/task
  launch
  models
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multibot_server.dsv.in")

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_ros)

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