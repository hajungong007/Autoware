cmake_minimum_required(VERSION 2.8.3)
project(object_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  tf
  sensor_msgs
  nav_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp pcl_ros pcl_conversions tf sensor_msgs nav_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(laserscan2costmap nodes/laserscan2costmap/laserscan2costmap.cpp)
target_link_libraries(laserscan2costmap ${catkin_LIBRARIES})

add_executable(points2costmap nodes/points2costmap/points2costmap.cpp)
target_link_libraries(points2costmap ${catkin_LIBRARIES})
