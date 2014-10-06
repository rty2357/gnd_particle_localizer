##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(gnd_particle_localizer)

##############################################################################
# Catkin
##############################################################################

find_package(catkin
	REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	gnd_geometry2d_msgs
	gnd_rosutil
	gndlib
	message_generation
)

##############################################################################
## Declare ROS messages and services 
##############################################################################

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
	msg_localization_particles2d_stamped.msg
	msg_localization_particle_weights_stamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "*.hpp"
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gnd_particle_localizer
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs gnd_geometry2d_msgs gnd_rosutil gndlib message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

##############################################################################
# Sources
##############################################################################

#add_library(gnd_particle_localizer src/lib/gnd_particle_localizer.cpp)
#target_link_libraries(gnd_particle_localizer ${catkin_LIBRARIES})

#install(TARGETS gnd_particle_localizer
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/gnd_particle_localizer/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")

##############################################################################
# Example
##############################################################################

add_executable(gnd_particle_localizer src/gnd_particle_localizer.cpp)
target_link_libraries(gnd_particle_localizer ${catkin_LIBRARIES})
add_dependencies(gnd_particle_localizer gnd_particle_localizer_generate_messages_cpp)
install(TARGETS gnd_particle_localizer 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

##############################################################################
# Test
##############################################################################

#if(CATKIN_ENABLE_TESTING)
#  find_package(catkin COMPONENTS rostest rosunit)
#  include_directories(${GTEST_INCLUDE_DIRS})
#  catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)
#  if(TARGET ${PROJECT_NAME}-test)
#    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  endif()
#endif()
