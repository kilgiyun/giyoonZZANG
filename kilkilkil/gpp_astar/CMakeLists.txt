cmake_minimum_required(VERSION 2.8.3)
project(gpp_astar)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  std_srvs
  dynamic_reconfigure
  rospy
)

add_service_files(
  FILES
  Astar.srv
  Astar_call.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
   DEPENDENCIES
   std_msgs
)

#generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES am_ugv_uav
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime rospy std_msgs # nodelet 
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/am_ugv_uav.cpp
# )

install(FILES 
  #list of necessary files (xml...)
  # settings.json
  # nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  launch
  #rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
