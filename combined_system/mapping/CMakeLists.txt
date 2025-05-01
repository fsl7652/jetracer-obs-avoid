cmake_minimum_required(VERSION 3.10.0)
project(mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
  laser_geometry
  message_filters
  eigen_conversions
)
message("==============================================")
message(${catkin_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mapping
  # CATKIN_DEPENDS message_generation   
  CATKIN_DEPENDS std_msgs roscpp rospy sensor_msgs nav_msgs
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( mapping 
    src/mapping.cpp 
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)

  catkin_add_gtest(mapping_test tests/mapping_test.cpp)
  target_link_libraries(mapping_test
    mapping 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${GTEST_LIBRARIES}
  )
endif()

target_link_libraries(mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES})

