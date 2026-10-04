cmake_minimum_required(VERSION 3.8)
project(lessien_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(EXEC wheel_odometry_node)
add_executable(${EXEC} src/wheel_odometry_node.cpp)
target_compile_features(${EXEC} PRIVATE cxx_std_17)
ament_target_dependencies(${EXEC}
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
target_include_directories(${EXEC} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(JS_EXEC wheel_joint_state_node)
add_executable(${JS_EXEC} src/wheel_joint_state_node.cpp)
target_compile_features(${JS_EXEC} PRIVATE cxx_std_17)
ament_target_dependencies(${JS_EXEC}
  rclcpp
  std_msgs
  sensor_msgs
)
target_include_directories(${JS_EXEC} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(TRIP_EXEC ekf_trip_distance_node)
add_executable(${TRIP_EXEC} src/ekf_trip_distance_node.cpp)
target_compile_features(${TRIP_EXEC} PRIVATE cxx_std_17)
ament_target_dependencies(${TRIP_EXEC}
  rclcpp
  std_msgs
  nav_msgs
  diagnostic_msgs
)
target_include_directories(${TRIP_EXEC} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS ${EXEC} ${JS_EXEC} ${TRIP_EXEC} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
