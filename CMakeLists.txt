cmake_minimum_required(VERSION 3.3)
project(seeker)

find_package(OpenCV REQUIRED)



if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin QUIET COMPONENTS roscpp)
elseif($ENV{ROS_VERSION} EQUAL 2)
  find_package(ament_cmake QUIET)
endif()

if (catkin_FOUND)
  message(STATUS "ROS *1* version found, building ROS1.cmake")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif (ament_cmake_FOUND)
  message(STATUS "ROS *2* version found, building ROS2.cmake")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
else ()
  message(STATUS "No ROS versions found, building ROS1.cmake")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
endif ()
