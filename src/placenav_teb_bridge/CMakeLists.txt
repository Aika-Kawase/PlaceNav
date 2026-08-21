cmake_minimum_required(VERSION 3.0.2)
project(placenav_teb_bridge)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  interactive_markers
  nav_msgs
  roscpp
  rospy
  teb_local_planner
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS costmap_2d geometry_msgs interactive_markers nav_msgs roscpp rospy
                 teb_local_planner tf2_ros visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(placenav_teb_bridge_node src/placenav_teb_bridge_node.cpp)
target_compile_features(placenav_teb_bridge_node PRIVATE cxx_std_14)
target_link_libraries(placenav_teb_bridge_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/publish_test_gnm_path.py
  scripts/publish_test_obstacle_cloud.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS placenav_teb_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch stage
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
