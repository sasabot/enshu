cmake_minimum_required(VERSION 2.8.3)
project(aero_mydemo)

find_package(catkin REQUIRED
  roscpp tf std_msgs sensor_msgs geometry_msgs roslib
  aero_std roboenvcv negomo_enshu aero_devel_lib
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES aero_mydemo_lib
 CATKIN_DEPENDS roscpp tf std_msgs sensor_msgs geometry_msgs roslib aero_std aero_devel_lib negomo_enshu
#  DEPENDS system_lib
)

add_definitions(-std=c++11)

add_library(aero_mydemo_lib
  lib/mydemo_lib.cc
  lib/handlePayment.cc
)

target_link_libraries(aero_mydemo_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(shop_interaction src/shop_interaction.cc)
target_link_libraries(shop_interaction aero_mydemo_lib ${catkin_LIBRARIES})
