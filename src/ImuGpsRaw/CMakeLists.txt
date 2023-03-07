cmake_minimum_required(VERSION 3.0.2)
project(ImuGpsRaw)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  serial
  std_msgs
  sensor_msgs
)


add_message_files(
  FILES
  GPS_Data.msg
  IMU_Data.msg
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ImuGpsRaw
 CATKIN_DEPENDS geometry_msgs roscpp rospy serial std_msgs message_runtime sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(demo src/demo.cpp)


add_dependencies(demo ${PROJECT_NAME}_generate_messages_cpp)


target_link_libraries(demo
  ${catkin_LIBRARIES}
)


find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
