cmake_minimum_required(VERSION 3.8)
project(hive_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

install(DIRECTORY
  config
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

ament_package()
