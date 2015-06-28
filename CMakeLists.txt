cmake_minimum_required(VERSION 2.8.3)
project(libbarrett_ros) 
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  joint_limits_interface
  roscpp
  transmission_interface
)

find_package(Boost REQUIRED)
find_package(Barrett REQUIRED)

# find_package(Barrett) does not match BARRETT_INCLUDE_DIRS, so we can't use
# DEPEND and have to manually pass BARRETT_INCLUDE_DIRS and BARRETT_LIBRARIES
# to the catkin_package() macro.
catkin_package(
  INCLUDE_DIRS
    include
    ${BARRETT_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    ${BARRETT_LIBRARIES}
  DEPENDS
    Boost
  CATKIN_DEPENDS 
    roscpp
    hardware_interface
    transmission_interface
)

include_directories(include)
include_directories(SYSTEM
  ${BARRETT_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${BARRETT_LIB_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)
add_definitions(
  ${BARRETT_DEFINITIONS}
  ${Boost_DEFINITIONS}
  ${catkin_DEFINITIONS}
)

# The version of Boost shipped with Ubuntu 12.04 predates Boost.Atomic, which
# as added in Boost 1.53. It's a header-only library, so we'll download it.
if (${Boost_VERSION} VERSION_LESS 1.53.0)
  message(STATUS "Found Boost ${Boost_VERSION}, which does not include"
                 " Boost.Atomic (requires 1.53.0 or greater).")
  message(STATUS "Downloading the Boost.Atomic 1.53.0 headers.")

  include(ExternalProject)
  ExternalProject_Add(Boost_atomic_EXTERNAL
    URL https://github.com/boostorg/atomic/archive/boost-1.53.0.tar.gz
    URL_MD5 9ab07a4f6611cd51d6da26b222d49bc9
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_DIR ""
  )
  ExternalProject_Get_Property(Boost_atomic_EXTERNAL SOURCE_DIR)
  include_directories(BEFORE SYSTEM ${SOURCE_DIR}/include)
else ()
 message(STATUS "Found Boost ${Boost_VERSION} (>= 1.53.0). Using the system"
                " version of Boost.Atomic.")
endif ()

add_library(${PROJECT_NAME}
  src/BarrettRobotHW.cpp
  src/Bus.cpp
  src/BusInfo.cpp
  src/ControlLoop.cpp
  src/Cycle.cpp
  src/ForceTorqueSensorHW.cpp 
  src/HandHW.cpp
  src/Schedule.cpp
  src/TaskSet.cpp
  src/tasks/AccelerationTask.cpp
  src/tasks/ForceTorqueTask.cpp
  src/tasks/WAMPositionTask.cpp
  src/tasks/WAMTorqueTask.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${BARRETT_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_controller
  src/controller.cpp
)
target_link_libraries(${PROJECT_NAME}_controller
  ${PROJECT_NAME}
  ${BARRETT_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_controller PROPERTIES
  OUTPUT_NAME controller
)
