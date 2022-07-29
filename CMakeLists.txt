cmake_minimum_required(VERSION 2.8.3)
project(alarm-light)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  smarthome
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(${PROJECT_NAME}_alarm-light src/alarm-light.cpp)
target_link_libraries(${PROJECT_NAME}_alarm-light ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_alarm-light PROPERTIES
  OUTPUT_NAME alarm-light
  PREFIX "")

install(TARGETS ${PROJECT_NAME}_alarm-light 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})