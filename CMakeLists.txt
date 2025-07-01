cmake_minimum_required(VERSION 3.8)
project(rviz_hmi_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Gamepad) # requires libqt5gamepad5-dev on Ubuntu (not available via rosdep!)

set(CMAKE_AUTOMOC ON)

set(SRC_FILES
  src/teleop_panel.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gamepad
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rviz_common"
  "rviz_rendering"
  "geometry_msgs"
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Gamepad_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(rviz_common plugin.xml)

ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  geometry_msgs
)

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

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
