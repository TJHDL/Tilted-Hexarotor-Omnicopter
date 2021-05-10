cmake_minimum_required(VERSION 3.0.2)
project(omnicopter_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs 
  mav_msgs 
  nav_msgs 
  sensor_msgs 
  cmake_modules 
  roscpp 
  rospy 
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES omnicopter_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen3
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(omnicopter_position_controller
  src/omnicopter_position_controller.cpp
)
add_dependencies(omnicopter_position_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(omnicopter_position_controller
  ${catkin_LIBRARIES}
)

add_executable(omnicopter_attitude_controller
  src/omnicopter_attitude_controller.cpp
)
add_dependencies(omnicopter_attitude_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(omnicopter_attitude_controller
  ${catkin_LIBRARIES}
)

add_executable(omnicopter_body_angular_velocity_controller
  src/omnicopter_body_angular_velocity_controller.cpp
)
add_dependencies(omnicopter_body_angular_velocity_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(omnicopter_body_angular_velocity_controller
  ${catkin_LIBRARIES}
)

add_executable(omnicopter_control_allocator
  src/omnicopter_control_allocator.cpp
)
add_dependencies(omnicopter_control_allocator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(omnicopter_control_allocator
  ${catkin_LIBRARIES}
)

add_library(position_controller
  include/omnicopter_controller/omnicopter_position_controller.h
)
add_dependencies(omnicopter_position_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(omnicopter_position_controller
  ${catkin_LIBRARIES}
)
set_target_properties(position_controller PROPERTIES LINKER_LANGUAGE C++)

add_library(attitude_controller
  include/omnicopter_controller/omnicopter_attitude_controller.h
)
add_dependencies(omnicopter_attitude_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(omnicopter_attitude_controller
  ${catkin_LIBRARIES}
)
set_target_properties(attitude_controller PROPERTIES LINKER_LANGUAGE C++)

add_library(body_angular_velocity_controller
  include/omnicopter_controller/omnicopter_body_angular_velocity_controller.h
)
add_dependencies(omnicopter_body_angular_velocity_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(omnicopter_body_angular_velocity_controller
  ${catkin_LIBRARIES}
)
set_target_properties(body_angular_velocity_controller PROPERTIES LINKER_LANGUAGE C++)

add_library(control_allocator
  include/omnicopter_controller/omnicopter_control_allocator.h
)
add_dependencies(omnicopter_control_allocator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(omnicopter_control_allocator
  ${catkin_LIBRARIES}
)
set_target_properties(control_allocator PROPERTIES LINKER_LANGUAGE C++)
