cmake_minimum_required(VERSION 2.8.3)
project(combined_system)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")
if(CATKIN_ENABLE_TESTING)
  message(STATUS "Building with coverage flags")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0 -fprofile-arcs -ftest-coverage")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()


find_package(OpenMP REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(ompl REQUIRED)

set(PLAN_UTILS_INCLUDE_DIRS include/plan_utils) 
set(MAPPING_INCLUDE_DIR mapping/include)
set(MAPPING_SRC_DIR mapping/src)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    decomp_ros_utils
    tf
    laser_geometry
    pcl_conversions
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${DECOMP_UTIL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${MAPPING_INCLUDE_DIR}
)

# Find all mapping source files
file(GLOB MAPPING_SOURCES "${MAPPING_SRC_DIR}/*.cpp")


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS 
        include 
        ${CMAKE_CURRENT_BINARY_DIR}
        ${PLAN_UTILS_INCLUDE_DIRS}
        ${MAPPING_INCLUDE_DIR}
    CATKIN_DEPENDS 
        decomp_ros_utils
        roscpp
        visualization_msgs
        std_msgs
        sensor_msgs
        geometry_msgs
)


# Create mapping library
add_library(mapping_lib ${MAPPING_SOURCES})
target_link_libraries(mapping_lib
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)

add_library(kino_astar
    src/kino_astar.cpp
    src/raycast.cpp
)

add_library(traj_optimizer
    src/traj_optimizer.cpp
)

add_library(traj_manager
    src/traj_manager.cpp
    src/traj_optimizer.cpp
)

# Main planner executable
add_executable(state_machine
    src/state_machine.cpp
    src/obs_avoid_fsm.cpp
    ${PROTO_SRCS}
)

add_executable(model_test
    src/model_test.cpp
)

target_link_libraries(state_machine
    mapping_lib
    kino_astar
    traj_optimizer
    traj_manager
    ompl
    "${OpenMP_CXX_FLAGS}"
    ${catkin_LIBRARIES}
    ${Protobuf_LIBRARIES}
)


target_link_libraries(state_machine
    mapping_lib
    ompl
    "${OpenMP_CXX_FLAGS}"
    ${catkin_LIBRARIES}
    ${Protobuf_LIBRARIES}
)

target_link_libraries(model_test
    ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/run_py3_venv.sh
  scripts/ros_road_follow.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)

  catkin_add_gtest(manager_test tests/manager_test.cpp)
  target_link_libraries(manager_test
    mapping_lib
    traj_manager
    traj_optimizer
    kino_astar
    ompl
    ${catkin_LIBRARIES}
    ${Protobuf_LIBRARIES}
    ${PCL_LIBRARIES}
    ${GTEST_LIBRARIES}
  )

  catkin_add_gtest(optimizer_test tests/optimizer_test.cpp)
  target_link_libraries(optimizer_test
    mapping_lib
    traj_optimizer
    ${catkin_LIBRARIES}
    ${Protobuf_LIBRARIES}
    ${GTEST_LIBRARIES}
  )
  
  catkin_add_gtest(kino_test tests/kino_test.cpp)
  target_link_libraries(kino_test
    mapping_lib
    kino_astar
    ompl
    ${catkin_LIBRARIES}
    ${Protobuf_LIBRARIES}
    ${GTEST_LIBRARIES}
  )
  

endif()