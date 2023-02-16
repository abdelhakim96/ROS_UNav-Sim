cmake_minimum_required(VERSION 2.8.3)
project(bluerov2_trajectory)
set(PROJECT_NAME bluerov2_trajectory)
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

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs roslib nav_msgs mavros_msgs tf dynamic_reconfigure message_generation
)

generate_dynamic_reconfigure_options(cfg/set_trajectory.cfg cfg/set_recdata.cfg cfg/set_wind_generation.cfg)

find_package(Eigen3 3.0.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



add_message_files(
  FILES
  CurrentDist.msg
)



generate_messages(
   DEPENDENCIES
  std_msgs  # Or other packages containing msgs
 )



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf geometry_msgs mavros_msgs message_runtime
)







#############
# LIBRARIES #
#############

if(true)

include_directories(
  ${catkin_INCLUDE_DIRS}

#  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)


############
# BINARIES #
############

add_executable(rov_trajectory src/trajectory.cpp)
target_link_libraries(rov_trajectory ${catkin_LIBRARIES})
add_dependencies(rov_trajectory ${PROJECT_NAME}_gencfg)



add_executable(recdata src/recdata.cpp)
target_link_libraries(recdata ${catkin_LIBRARIES})
add_dependencies(recdata ${PROJECT_NAME}_gencfg)


add_executable(wind_generation src/wind_generation.cpp)
target_link_libraries(wind_generation ${catkin_LIBRARIES})
add_dependencies(wind_generation ${PROJECT_NAME}_gencfg)


endif()
