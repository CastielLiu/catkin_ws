cmake_minimum_required(VERSION 2.8.3)
project(ps_gps)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  roslaunch
 
)

roslaunch_add_file_check(launch)

add_message_files( FILES   gps.msg  )
generate_messages( DEPENDENCIES   std_msgs )

catkin_package(   INCLUDE_DIRS include   LIBRARIES ps_gps   CATKIN_DEPENDS roscpp rospy std_msgs
   DEPENDS system_lib
)

include_directories(  include  ${catkin_INCLUDE_DIRS})

add_executable(ps_serial src/ps_serial.cpp src/serial_open.cpp src/ps_gps_func.cpp)
add_dependencies(ps_serial ps_gps_generate_messages_cpp)
target_link_libraries(ps_serial    ${catkin_LIBRARIES}  )

add_executable(ps_receive src/ps_receive.cpp)
add_dependencies(ps_receive ps_gps_generate_messages_cpp)
target_link_libraries(ps_receive    ${catkin_LIBRARIES}  )
