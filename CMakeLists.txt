cmake_minimum_required(VERSION 2.8.3)
project(pal_face_detector_opencv)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport    
  pal_detection_msgs
  sensor_msgs
  roscpp
####################/////////////
  actionlib
  geometry_msgs
  control_msgs
  tf
####################/////////////
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


##########
# Flags ##
##########

#add_definitions(-DPAL_MODULE_NAME=\"VISION\")
#add_definitions(-DPAL_PROJECT_NAME=\"${PROJECT_NAME}\")
#add_definitions(-DPAL_LIBRARY_NAME=\"${PROJECT_NAME}\")

###################################
## catkin specific configuration ##
###################################


catkin_package(
  CATKIN_DEPENDS sensor_msgs pal_detection_msgs tf
)

include_directories(SYSTEM
${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

## Declare a cpp executable
add_executable(pal_face_detector_opencv src/face_detector.cpp)
target_link_libraries(pal_face_detector_opencv ${catkin_LIBRARIES})
add_dependencies(pal_face_detector_opencv pal_detection_msgs_generate_messages_cpp)

####################/////////////
add_executable(head_action src/head_action.cpp)
target_link_libraries(head_action
  ${catkin_LIBRARIES})

add_executable(transform_listener src/transform_listener.cpp)
target_link_libraries(transform_listener
  ${catkin_LIBRARIES})
####################/////////////


#############
## Install ##
#############

install(TARGETS pal_face_detector_opencv
		####################/////////////
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  		####################/////////////
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
