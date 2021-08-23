cmake_minimum_required(VERSION 3.0.2)
project(drone_ball_catcher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  mavros_msgs
  mavros
  tf2
  tf2_ros
  tf2_geometry_msgs
  gazebo_msgs
)

find_package(Boost REQUIRED COMPONENTS 
  system
)


################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drone_ball_catcher
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
  DEPENDS  
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library


add_executable(ball_locator src/ball_locator.cpp)
add_executable(ball_image_filter src/ball_image_filter.cpp)
add_executable(ball_trajectory src/ball_trajectory.cpp)
add_executable(drone_test_AB src/drone_test_no_yaw.cpp)
add_executable(drone_test_C src/drone_test_yaw.cpp)
add_executable(drone_test_DE src/drone_test_yaw_predict.cpp)
add_executable(ball_tf src/ball_tf.cpp)
add_executable(ball_pose src/ball_pose_from_bag.cpp)
add_executable(droneballpose_sub src/gazebo_models_sub.cpp)

 add_dependencies(ball_locator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(ball_image_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(ball_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(drone_test_AB ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(drone_test_C ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(drone_test_DE ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(ball_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(ball_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(droneballpose_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
 target_link_libraries(ball_locator   ${catkin_LIBRARIES} ${boost_LIBRARIES})
 target_link_libraries(ball_image_filter   ${catkin_LIBRARIES} ${boost_LIBRARIES})
 target_link_libraries(ball_trajectory   ${catkin_LIBRARIES} ${boost_LIBRARIES})
 target_link_libraries(drone_test_AB ${catkin_LIBRARIES} ${boost_LIBRARIES})
 target_link_libraries(drone_test_C ${catkin_LIBRARIES} ${boost_LIBRARIES})
 target_link_libraries(drone_test_DE ${catkin_LIBRARIES} ${boost_LIBRARIES})
 target_link_libraries(ball_tf ${catkin_LIBRARIES} ${boost_LIBRARIES})
 target_link_libraries(ball_pose ${catkin_LIBRARIES} ${boost_LIBRARIES})
 target_link_libraries(droneballpose_sub ${catkin_LIBRARIES} ${boost_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############

