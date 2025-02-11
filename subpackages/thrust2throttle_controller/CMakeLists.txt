cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME thrust2throttle_controller)
project(${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  mavros_msgs
  mav_msgs
  robot_process
  tf
  ros_utils_lib
)

catkin_package(
  INCLUDE_DIRS src/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs mavros_msgs robot_process mav_msgs tf ros_utils_lib
  )

include_directories(
  src/include
  ${catkin_INCLUDE_DIRS}
)


set(CPP_PROJECT_FILES 
  src/source/thrust2throttle_controller.cpp
)

set(HEADER_PROJECT_FILES
  src/include/thrust2throttle_controller.hpp
)

# add_library(${PROJECT_NAME}
#   ${CPP_PROJECT_FILES}
#   ${HEADER_PROJECT_FILES}
# )
# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/source/thrust2throttle_controller_node.cpp  ${CPP_PROJECT_FILES} ${HEADER_PROJECT_FILES}) 
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})