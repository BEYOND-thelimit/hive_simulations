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
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(sensor_noise_maker src/sensor_noise_maker.cpp)
ament_target_dependencies(sensor_noise_maker
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(depth_sub src/depth_sub.cpp)
ament_target_dependencies(depth_sub 
  rclcpp 
  std_msgs
  nav_msgs
  sensor_msgs 
  PCL 
  pcl_conversions 
  OpenCV
)

# node
install(TARGETS sensor_noise_maker
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
	depth_sub
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
