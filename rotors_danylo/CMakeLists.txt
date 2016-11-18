cmake_minimum_required(VERSION 2.8.3)
project(rotors_danylo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES lqr_controller # frequency_probe_node
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(lqr_controller
  src/library/lqr_controller
)

target_link_libraries(lqr_controller ${catkin_LIBRARIES})
add_dependencies(lqr_controller ${catkin_EXPORTED_TARGETS})

add_executable(lqr_controller_node src/nodes/lqr_controller_node.cpp)
add_dependencies(lqr_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lqr_controller_node
  lqr_controller ${catkin_LIBRARIES})

add_executable(frequency_probe_node src/nodes/frequency_probe_node.cpp)
add_dependencies(frequency_probe_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(frequency_probe_node ${catkin_LIBRARIES})

install(TARGETS lqr_controller #list all controllers here
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS lqr_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING REGEX "^.*\\.\\b(h|hpp)\\b$"
)
