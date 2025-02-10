cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME pixhawk_interface)
project(${PROJECT_NAME})
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
  std_msgs
  sensor_msgs
  mav_msgs
  geometry_msgs
  mavros
  mavros_msgs
  robot_process
  aerostack_msgs
  ros_utils_lib
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs mav_msgs  geometry_msgs mavros mavros_msgs robot_process aerostack_msgs ros_utils_lib
)


set(HEADER_PATH src/include)
set(SRC_PATH src/sources)
set(NODES_PATH src/nodes)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${HEADER_PATH}
)

## picks all the headers files 
file(GLOB LIB_HEADER_FILES 
	${HEADER_PATH}/*.hpp
	${HEADER_PATH}/*.h	
)

## picks all the source files 
file(GLOB LIB_SOURCE_FILES
	${SRC_PATH}/*.cpp
)

add_library(${PROJECT_NAME}
  ${LIB_HEADER_FILES}
  ${LIB_SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#state interface
add_executable(${PROJECT_NAME}_state_node ${NODES_PATH}/pixhawk_state_node.cpp)
target_link_libraries(${PROJECT_NAME}_state_node ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_state_node ${catkin_EXPORTED_TARGETS})

#command interface
add_executable(${PROJECT_NAME}_command_node ${NODES_PATH}/pixhawk_command_node.cpp)
target_link_libraries(${PROJECT_NAME}_command_node ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_command_node ${catkin_EXPORTED_TARGETS})


#test node
# add_executable(${PROJECT_NAME}_takeoff_node ${NODES_PATH}/test_takeoff.cpp)
# target_link_libraries(${PROJECT_NAME}_takeoff_node ${catkin_LIBRARIES} ${PROJECT_NAME})
# add_dependencies(${PROJECT_NAME}_takeoff_node ${catkin_EXPORTED_TARGETS})


# #fake state estimation
# add_executable(${PROJECT_NAME}_fake_estimation_node ${NODES_PATH}/fake_state_estimation_test.cpp)
# target_link_libraries(${PROJECT_NAME}_fake_estimation_node ${catkin_LIBRARIES} ${PROJECT_NAME})
# add_dependencies(${PROJECT_NAME}_fake_estimation_node ${catkin_EXPORTED_TARGETS})
