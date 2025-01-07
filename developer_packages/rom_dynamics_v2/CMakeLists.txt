cmake_minimum_required(VERSION 3.8)
project(rom_dynamics_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# find dependencies
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)


find_package(Qt5 REQUIRED COMPONENTS Widgets)
#find_package(Qt6 COMPONENTS Core Widgets REQUIRED)
#qt_standard_project_setup()

#---------------- အင်ကလု ----------------#
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ros2/header)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ros2/source)

set(CMAKE_AUTOUIC_SEARCH_PATHS src)
set(UI_FILES src/mainwindow.ui)
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

add_executable(ROM_DYNAMICS
    src/mainwindow.h
    src/main.cpp
    src/mainwindow.cpp
    ros2/header/cmd_publisher.h
    ros2/source/cmd_publisher.cpp
    ros2/header/subscriber.h
    ros2/source/subscriber.cpp
    ros2/header/map_subscriber.h
    ros2/source/map_subscriber.cpp
    ${UI_HEADERS}
)


target_link_libraries(ROM_DYNAMICS ${Qt5Core_LIBRARIES})
target_link_libraries(ROM_DYNAMICS ${Qt5Widgets_LIBRARIES})
ament_target_dependencies(ROM_DYNAMICS rclcpp std_msgs geometry_msgs nav_msgs)

#---------------- END -----------------#

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
