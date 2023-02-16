cmake_minimum_required(VERSION 2.8.3)
project(bluerov2_nmpc)
set(PROJECT_NAME bluerov2_nmpc)
set(ROS_BUILD_TYPE Debug)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ${CMAKE_CXX_FLAGS_DEBUG} -O1")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs roslib nav_msgs mavros_msgs tf)

find_package(Eigen3 3.0.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)





#############
# LIBRARIES #
#############

add_library(${PROJECT_NAME}_LIBRARY
#src/nmpc_pc.cpp
  solver/qpoases/SRC/Bounds.cpp
  solver/qpoases/SRC/Constraints.cpp
  solver/qpoases/SRC/CyclingManager.cpp
  solver/qpoases/SRC/Indexlist.cpp
  solver/qpoases/SRC/MessageHandling.cpp
  solver/qpoases/SRC/QProblem.cpp
  solver/qpoases/SRC/QProblemB.cpp
  solver/qpoases/SRC/SubjectTo.cpp
  solver/qpoases/SRC/Utils.cpp
  solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  solver/nmpc_qpoases_interface.cpp
  solver/nmpc_integrator.c
  solver/nmpc_solver.c
  solver/nmpc_auxiliary_functions.c
)

include_directories(${PROJECT_NAME}_LIBRARY PRIVATE
  ${catkin_INCLUDE_DIRS}

  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include

  ${PROJECT_SOURCE_DIR}/solver
  ${PROJECT_SOURCE_DIR}/solver/qpoases
  ${PROJECT_SOURCE_DIR}/solver/qpoases/INCLUDE
  ${PROJECT_SOURCE_DIR}/solver/qpoases/INCLUDE/EXTRAS
  ${PROJECT_SOURCE_DIR}/solver/qpoases/SRC

)




add_message_files(
  FILES
  PwmCmd.msg

)

generate_messages(
   DEPENDENCIES
  std_msgs  # Or other packages containing msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bluerov2_nmpc
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


############
# BINARIES #
############

add_executable(bluerov2_nmpc src/nmpc_bluerov2_2D_main.cpp src/nmpc_bluerov2_2D.cpp )
target_link_libraries(bluerov2_nmpc ${PROJECT_NAME}_LIBRARY ${catkin_LIBRARIES})

#catkin_install_python(PROGRAMS scripts/thrust_allocator.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


catkin_install_python(PROGRAMS scripts/thrust_allocator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


