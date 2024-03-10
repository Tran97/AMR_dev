cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_ros
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(grid_to_occ src/grid_to_occ.cpp)
target_link_libraries(grid_to_occ ${catkin_LIBRARIES})
add_dependencies(grid_to_occ ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS 
  script/dijkstra_planner.py
  script/dijkstra_planner_custom.py
  script/publish_goal.py
  script/path_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)