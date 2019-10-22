cmake_minimum_required(VERSION 3.5)
project(robobo_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(robobo_msgs REQUIRED)

include_directories(include 
 	${gazebo_dev_INCLUDE_DIRS}
	${gazebo_ros_INCLUDE_DIRS}
	${robobo_msgs_INCLUDE_DIRS}
)

link_directories(${gazebo_dev_LIBRARY_DIRS} ${robobo_msgs_LIBRARY_DIRS} )

add_library(move_wheels SHARED plugins/move_wheels.cpp)
ament_target_dependencies(move_wheels
	"gazebo_dev"
	"gazebo_ros"
	"robobo_msgs"
	"rclcpp"
)
ament_export_libraries(move_wheels)

add_library(move_pan_tilt SHARED plugins/encoders.cpp)
ament_target_dependencies(move_pan_tilt
	"gazebo_dev"
	"gazebo_ros"
	"robobo_msgs"
	"rclcpp"
)
ament_export_libraries(move_pan_tilt)

add_library(encoders SHARED plugins/encoders.cpp)
ament_target_dependencies(encoders
	"gazebo_dev"
	"gazebo_ros"
	"robobo_msgs"
	"rclcpp"
)
ament_export_libraries(encoders)

add_library(robobo_imu_sensor SHARED plugins/robobo_imu_sensor.cpp)
ament_target_dependencies(robobo_imu_sensor
	"gazebo_dev"
	"gazebo_ros"
	"robobo_msgs"
	"rclcpp"
)
ament_export_libraries(robobo_imu_sensor)

#ament_export_include_directories(include)
#ament_export_dependencies(rclcpp rclpy gazebo_ros gazebo_plugins)
#ament_export_libraries(move_wheels move_pan_tilt encoders 
#	infrared_range robobo_camera_utils robobo_camera robobo_imu_sensor)

install(TARGETS
	move_wheels
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin)

install(TARGETS
	move_pan_tilt
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin)

install(TARGETS
	encoders
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin)

install(TARGETS
	robobo_imu_sensor
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin)

install(TARGETS
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models worlds launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/spawn_robobo.py
  scripts/robobo_validation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()