cmake_minimum_required(VERSION 2.8.3)
project(epos2_test)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES epos2_test
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(epos2_test src/epos2_test.cpp)
target_link_libraries(epos2_test ${catkin_LIBRARIES})

add_executable(motor_control src/epos2_control.cpp include/epos2_test/epos2_control.cpp)
target_link_libraries(motor_control ${catkin_LIBRARIES})