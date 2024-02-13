cmake_minimum_required(VERSION 3.8)
project(test_markers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#find_package( REQUIRED)

add_executable(plane src/plane.cpp)
ament_target_dependencies(plane rclcpp visualization_msgs )

add_executable(move_plane src/move_plane.cpp)
ament_target_dependencies(move_plane rclcpp visualization_msgs interactive_markers )

add_executable(free_move_plane src/free_move_plane.cpp)
ament_target_dependencies(free_move_plane rclcpp visualization_msgs interactive_markers )

add_executable(menu_plane src/menu_plane.cpp)
ament_target_dependencies(menu_plane rclcpp visualization_msgs interactive_markers )

add_executable(point_menu_plane src/point_menu_plane.cpp)
ament_target_dependencies(point_menu_plane rclcpp visualization_msgs interactive_markers geometry_msgs )

# Install executable
install(TARGETS
  plane
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  move_plane
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  free_move_plane
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  menu_plane
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  point_menu_plane
  DESTINATION lib/${PROJECT_NAME})
  
  





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
