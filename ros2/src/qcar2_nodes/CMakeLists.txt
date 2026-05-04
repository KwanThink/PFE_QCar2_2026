cmake_minimum_required(VERSION 3.8)
project(qcar2_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(qcar2_interfaces REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

# Need to define _NVIDIA macro since the Quanser SDK has different
# functionalities depending on which platform it is being used.
add_compile_definitions(_NVIDIA)

# The header 'cv_bridge/cv_bridge.hpp' was added in version 3.3.0.
# For older cv_bridge versions, use 'cv_bridge/cv_bridge.h'.
if(${cv_bridge_VERSION} VERSION_GREATER_EQUAL "3.3.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCV_BRDIGE_HAS_HPP")
endif()

# -----------------------------------------------------------------------------
# Trajectory generation / visualization
# Current portfolio milestone:
#   - Bezier-based trajectory generation
#   - Trajectory visualization in RViz
# -----------------------------------------------------------------------------
add_executable(qcar2_bezier src/qcar2_bezier.cpp)
ament_target_dependencies(qcar2_bezier
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
)

# -----------------------------------------------------------------------------
# NLMPC controller with ACADO/acados solver
#
# Temporarily disabled for portfolio submission.
#
# Reason:
#   This part is still under development. The current GitHub version focuses on
#   the completed trajectory generation / visualization milestone.
#
# Future work:
#   - Enable NLMPC trajectory tracking
#   - Integrate generated ACADO/acados solver
#   - Track Bezier/reference trajectory using optimal control
# -----------------------------------------------------------------------------

# add_executable(qcar2_nlmpc src/qcar2_nlmpc.cpp)
# ament_target_dependencies(qcar2_nlmpc
#   rclcpp
#   std_msgs
#   geometry_msgs
#   tf2
#   tf2_ros
# )

# set(ACADO_SOLVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/acado_qcar2_nlmpc)
# if(EXISTS ${ACADO_SOLVER_DIR}/acado_common.h)
#   message(STATUS "Found generated ACADO solver at ${ACADO_SOLVER_DIR}")
#   file(GLOB_RECURSE ACADO_SOLVER_SOURCES CONFIGURE_DEPENDS
#     ${ACADO_SOLVER_DIR}/*.c
#     ${ACADO_SOLVER_DIR}/*.cpp
#   )
#   list(FILTER ACADO_SOLVER_SOURCES EXCLUDE REGEX ".*/test.*\\.cpp$")
#   list(FILTER ACADO_SOLVER_SOURCES EXCLUDE REGEX ".*/make_.*\\.cpp$")
#
#   add_library(qcar2_acado_solver STATIC ${ACADO_SOLVER_SOURCES})
#   target_include_directories(qcar2_acado_solver PUBLIC
#     ${ACADO_SOLVER_DIR}
#     ${ACADO_SOLVER_DIR}/qpoases
#     ${ACADO_SOLVER_DIR}/qpoases/INCLUDE
#     ${ACADO_SOLVER_DIR}/qpoases/SRC
#   )
#   target_compile_options(qcar2_acado_solver PRIVATE -w)
#   target_link_libraries(qcar2_acado_solver m)
#
#   target_compile_definitions(qcar2_nlmpc PRIVATE QCAR2_USE_ACADO)
#   target_include_directories(qcar2_nlmpc PRIVATE ${ACADO_SOLVER_DIR})
#   target_link_libraries(qcar2_nlmpc qcar2_acado_solver)
# else()
#   message(WARNING "Generated ACADO solver not found at ${ACADO_SOLVER_DIR}. qcar2_nlmpc will build but will not run the solver.")
# endif()

# Optional helper target for exporting ACADO code.
# Usage after sourcing ACADO environment:
#   colcon build --packages-select qcar2_nodes --cmake-args -DBUILD_QCAR2_ACADO_EXPORTER=ON
#   ros2 run qcar2_nodes qcar2_acado_export
#
# Temporarily disabled for portfolio submission.

# option(BUILD_QCAR2_ACADO_EXPORTER "Build helper executable that exports the ACADO solver" OFF)
# if(BUILD_QCAR2_ACADO_EXPORTER)
#   if(DEFINED ENV{ACADO_ROOT})
#     add_executable(qcar2_acado_export src/qcar2_acado_export.cpp)
#     target_include_directories(qcar2_acado_export PRIVATE
#       $ENV{ACADO_ROOT}
#       $ENV{ACADO_ROOT}/acado
#       $ENV{ACADO_ROOT}/include
#       $ENV{ACADO_ROOT}/build
#     )
#     target_link_directories(qcar2_acado_export PRIVATE
#       $ENV{ACADO_ROOT}/build/lib
#       $ENV{ACADO_ROOT}/lib
#     )
#     target_link_libraries(qcar2_acado_export acado_toolkit_s)
#   else()
#     message(WARNING "BUILD_QCAR2_ACADO_EXPORTER=ON but ACADO_ROOT is not set. Source acado_env.sh first.")
#   endif()
# endif()

# -----------------------------------------------------------------------------
# QCar2 hardware and sensor nodes
# -----------------------------------------------------------------------------

# add_executable(qcar2_driver_interface src/qcar2_driver_interface.cpp)
# target_link_libraries(qcar2_driver_interface -lquanser_communications)
# ament_target_dependencies(qcar2_driver_interface rclcpp std_msgs geometry_msgs sensor_msgs control_msgs)

add_executable(command src/command.cpp)
target_link_libraries(command
  -lquanser_devices
  -lquanser_communications
  -lquanser_runtime
  -lquanser_common
)
ament_target_dependencies(command rclcpp geometry_msgs qcar2_interfaces)

add_executable(qcar2_odometry src/qcar2_odometry.cpp)
ament_target_dependencies(qcar2_odometry
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(qcar2_hardware src/qcar2_hardware.cpp)
target_link_libraries(qcar2_hardware
  -lhil
  -lquanser_devices
  -lquanser_runtime
  -lquanser_common
)
ament_target_dependencies(qcar2_hardware
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
  qcar2_interfaces
)

add_executable(rgbd src/rgbd.cpp)
target_include_directories(rgbd PUBLIC /usr/include/opencv4)
target_link_libraries(rgbd
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_imgcodecs
  -lquanser_media
  -lquanser_runtime
  -lquanser_common
)
ament_target_dependencies(rgbd
  rclcpp
  cv_bridge
  image_transport
  OpenCV
  std_msgs
  sensor_msgs
)

add_executable(lidar src/lidar.cpp)
target_link_libraries(lidar
  -lquanser_devices
  -lquanser_communications
  -lquanser_runtime
  -lquanser_common
)
ament_target_dependencies(lidar rclcpp sensor_msgs)

add_executable(csi src/csi.cpp)
target_include_directories(csi PUBLIC /usr/include/opencv4)
target_link_libraries(csi
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_imgcodecs
  -lquanser_media
  -lquanser_runtime
  -lquanser_common
)
ament_target_dependencies(csi
  rclcpp
  cv_bridge
  image_transport
  OpenCV
  std_msgs
  sensor_msgs
)

# -----------------------------------------------------------------------------
# TF nodes
# qcar2 -> lidar
# -----------------------------------------------------------------------------
add_executable(fixed_lidar_frame src/fixed_lidar_frame.cpp)
ament_target_dependencies(fixed_lidar_frame geometry_msgs rclcpp tf2_ros)

add_executable(fixed_lidar_frame_virtual src/fixed_lidar_frame_virtual.cpp)
ament_target_dependencies(fixed_lidar_frame_virtual geometry_msgs rclcpp tf2_ros)

# -----------------------------------------------------------------------------
# Nav2 to QCar2 command converter
# Converts Nav2 velocity commands into QCar2 motor commands.
# -----------------------------------------------------------------------------
add_executable(nav2_qcar2_converter src/nav2_qcar_command_convert.cpp)
ament_target_dependencies(nav2_qcar2_converter
  rclcpp
  geometry_msgs
  qcar2_interfaces
)

# -----------------------------------------------------------------------------
# Install nodes
# -----------------------------------------------------------------------------

# install(TARGETS qcar2_driver_interface
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS qcar2_bezier
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS qcar2_hardware
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS rgbd
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS lidar
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS csi
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS command
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS nav2_qcar2_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS qcar2_odometry
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS fixed_lidar_frame
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS fixed_lidar_frame_virtual
  DESTINATION lib/${PROJECT_NAME}
)

# NLMPC install disabled temporarily.
# install(TARGETS qcar2_nlmpc
#   DESTINATION lib/${PROJECT_NAME}
# )

# if(TARGET qcar2_acado_export)
#   install(TARGETS qcar2_acado_export
#     DESTINATION lib/${PROJECT_NAME}
#   )
# endif()

# -----------------------------------------------------------------------------
# Install launch, config, and model files
# -----------------------------------------------------------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY rt_models
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# -----------------------------------------------------------------------------
# Testing / linting
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip copyright checks for academic/portfolio submission.
  # Enable these again before production release.
  set(ament_cmake_copyright_FOUND TRUE)

  # Skip cpplint for now.
  # Enable this again once all files follow the final style guide.
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()