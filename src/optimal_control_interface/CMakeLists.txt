cmake_minimum_required(VERSION 2.8.3)
project(optimal_control_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  roslaunch
  std_msgs
  std_srvs
  uav_abstraction_layer
  nav_msgs
  uav_path_manager
  multidrone_msgs
)
find_package(PythonLibs 2.7)

catkin_package(
 INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy tf std_msgs std_srvs uav_abstraction_layer uav_path_manager multidrone_msgs
)

#roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)


include_directories(
  include
  lib
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)


set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/lib/libFORCESNLPsolver.so)



add_library(optimal_control_interface_library src/optimal_control_interface.cpp 
src/FORCESNLPsolver_casadi2forces.c 
src/FORCESNLPsolver_model_1.c
src/FORCESNLPsolver_model_30.c
)


add_executable(optimal_control_interface_node src/main.cpp)


target_link_libraries(optimal_control_interface_node optimal_control_interface_library)


target_link_libraries(optimal_control_interface_node 
${catkin_LIBRARIES} ${EXTRALIB_BIN} ${PYTHON_LIBRARIES})

add_dependencies(optimal_control_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})