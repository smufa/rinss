cmake_minimum_required(VERSION 2.8.3)
project(task2)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf2
	tf2_geometry_msgs
	pcl_conversions
  	pcl_msgs
	dynamic_reconfigure
)

find_package(OpenCV)

## Add include directories
include_directories(
	${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

link_libraries(${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_message_files(DIRECTORY msg
	FILES
  	Face.msg
	GreetingDelta.msg
	Greet.msg
	Cylinder.msg
)

add_service_files(
  FILES
  ColorClassifierService.srv
)

generate_messages(DEPENDENCIES 
	std_msgs
	geometry_msgs
	sensor_msgs
)

## Declare the catkin package
catkin_package(
  LIBRARIES
    pcl_ros_filters
    pcl_ros_io
    pcl_ros_tf
  CATKIN_DEPENDS
  	message_runtime
    dynamic_reconfigure
    pcl_conversions
    pcl_msgs
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS
    Boost
    Eigen
    PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(cylinder_segmentation src/cylinder_segmentation.cpp)
target_link_libraries(cylinder_segmentation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

