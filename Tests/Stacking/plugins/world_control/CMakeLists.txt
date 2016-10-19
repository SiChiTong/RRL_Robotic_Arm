cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x ${GAZEBO_CXX_FLAGS}")

add_library(armworld_plugin SHARED armworld_plugin.cc template.cc ArmController.cc InverseKinematics.cc Parameters.cc JointsController.cc Statistics.cc ArmStructure.cc ContactsManager.cc WorldView.cc GripperController.cc)
target_link_libraries(armworld_plugin
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTUBUF_LIBRARIES}
)
