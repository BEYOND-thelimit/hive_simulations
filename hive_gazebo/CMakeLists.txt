cmake_minimum_required(VERSION 3.8)
project(hive_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(odom_noise_maker_node src/odom_noise_maker.cpp)
ament_target_dependencies(odom_noise_maker_node
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# node
install(TARGETS odom_noise_maker_node
  DESTINATION lib/${PROJECT_NAME}
)

# Folder
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf/robot1
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf/robot2
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf/robot3
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf/simple_cameras
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf/intel_realsense
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
