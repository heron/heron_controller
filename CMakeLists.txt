cmake_minimum_required(VERSION 2.8.3)
project(heron_controller)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  heron_msgs
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/controller.cpp src/force_compensator)
add_dependencies(${PROJECT_NAME}_node heron_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node
                      PROPERTIES OUTPUT_NAME controller
                      PREFIX "")

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

