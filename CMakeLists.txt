cmake_minimum_required(VERSION 3.5.1)
project(mono_surface_pub)
set(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -pg -O2") #O0,O2: optimize 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge

  mavros_msgs
  
  message_generation

  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  message_filters
  image_transport
)

find_package(OpenCV REQUIRED)
# find_package(OpenCV 2 REQUIRED)

catkin_package(
  DEPENDS EIGEN3 #Eigen -> EIGEN3 : resolved
  # CATKIN_DEPENDS message_runtime roscpp message_filters
)

set(SRC ${SRC} 
  src/mono_surface.cpp
  src/mv_camera_calib.cpp
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

add_executable(mono_surface
  src/mono_surface.cpp)
target_link_libraries(mono_surface ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(mv_camera_calib
  src/mv_camera_calib.cpp)
target_link_libraries(mv_camera_calib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})