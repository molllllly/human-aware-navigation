cmake_minimum_required(VERSION 3.8)
project(air_navigation)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav2_dwb_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)

# SIP/Python stuff
#find_package(PythonInterp 3.0 REQUIRED)
#find_package(PythonLibrary 3.0 REQUIRED)
#find_package(SIP 4.18.0 REQUIRED)

find_package(PythonInterp 3.10 REQUIRED)
find_package(PythonLibrary 3.10 REQUIRED)
find_package(SIP 4.18.0 REQUIRED)

nav2_package()

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PYTHON_INCLUDE_PATH}
)

set(dependencies
  rclcpp
  nav2_dwb_controller
  pluginlib
  nav2_util
  nav2_costmap_2d
  )

message("LIBS: ${PYTHON_LIBRARY}")
message("DEPS: ${dependencies}")

add_library(air_navigation SHARED
        src/python_controller.cpp
        src/python_interpreter.cpp
        src/python_trajectory_critic.cpp
        src/python_trajectory_generator.cpp)
target_compile_definitions(air_navigation PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(air_navigation ${PYTHON_LIBRARY})
target_include_directories(air_navigation
  PUBLIC
  /opt/ros/humble/include/nav_2d_msgs
  /opt/ros/humble/include/dwb_msgs
)
ament_target_dependencies(air_navigation
  ${dependencies}
)

install(TARGETS air_navigation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


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

add_subdirectory(python)

ament_export_libraries(air_navigation)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core air_navigation_plugins.xml)
pluginlib_export_plugin_description_file(dwb_core air_navigation_plugins.xml)

ament_package()
