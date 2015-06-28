cmake_minimum_required(VERSION 2.8.3)
project(libbarrett_ros)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_manager
  hardware_interface
  joint_limits_interface
  roscpp
  transmission_interface
)

find_package(Boost REQUIRED)
find_package(Barrett REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BARRETT_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(
  ${BARRETT_LIB_DIRS}
  ${Boost_LIBRARY_DIRS}
)
add_definitions(
  ${BARRETT_DEFINITIONS}
)

add_executable(${PROJECT_NAME}
  src/controller.cpp
  src/tasks/AccelerationTask.cpp
  src/tasks/ForceTorqueTask.cpp
  src/tasks/WAMPositionTask.cpp
  src/tasks/WAMTorqueTask.cpp
  src/Cycle.cpp
  src/Schedule.cpp
  src/TaskSet.cpp
  src/BarrettRobotHW.cpp
  src/ForceTorqueSensorHW.cpp 
  src/HandHW.cpp
  src/BusInfo.cpp
  src/Bus.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${BARRETT_LIBRARIES}
  ${Boost_LIBRARIES}
)
