
cmake_minimum_required(VERSION 2.8.3)
project(image_depth_publisher_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)




find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


catkin_package(
 INCLUDE_DIRS include
  LIBRARIES image_depth_publisher_ros
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
  DEPENDS EIGEN3
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  
  ${EIGEN3_INCLUDE_DIR}
)
## Declare a C++ executable
add_executable(${PROJECT_NAME} src/image_depth_publisher_driver src/image_depth_publisher.cpp)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBRARIES}
)