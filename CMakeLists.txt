cmake_minimum_required(VERSION 2.8.3)
project(thermal_ros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
)

find_package( OpenCV REQUIRED )

include_directories(
	${catkin_INCLUDE_DIRS}
  include
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime cv_bridge sensor_msgs
  INCLUDE_DIRS
)

add_executable(${PROJECT_NAME}_node main.cpp)

target_link_libraries(${PROJECT_NAME}_node PUBLIC ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
${CMAKE_SOURCE_DIR}/libs/libUSBSDK.so
usb-1.0
)
