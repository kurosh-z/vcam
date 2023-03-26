cmake_minimum_required(VERSION 3.0.2)
project(vcam)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG")
set(ROSLINT_CPP_OPTS "--filter=+,-build/c++11")
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
roscpp
nodelet
dynamic_reconfigure
cv_bridge
image_transport
image_geometry
sensor_msgs
mavros_msgs
std_srvs
std_msgs
camera_calibration_parsers
camera_info_manager
message_generation
)
catkin_destinations()
add_definitions(-std=c++11 -Wall -g)
find_package(PkgConfig REQUIRED)
pkg_check_modules (libudev REQUIRED libudev)
pkg_check_modules(V4L REQUIRED libv4l2)
pkg_check_modules(UDEV REQUIRED libudev)
# pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_message_files(DIRECTORY msg
  FILES
  Exposure.msg
)
generate_messages(DEPENDENCIES std_msgs)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

#####################
set(VCAM_NODELET_NAME vcam_nodelet)
set(VCAM_LIB_NAME ecam_wrapper)

set(VCAM_LIB_SOURCES
  src/v4l2_cam_driver.cpp
  src/device.cpp
)

set(VCAM_LIB_HEADERS
  include/vcam/v4l2_cam_driver.h
  include/vcam/device.h
)

set(VCAM_NODELET_SOURCES
  src/vcam_nodelet.cpp
  # src/cam_driver.cpp 
  # src/device.cpp
)

set(VCAM_NODELET_HEADERS
  include/vcam/vcam_nodelet.hpp
  include/vcam/logging_macros.hpp
)

##### dynamic reconfiguration
generate_dynamic_reconfigure_options(
  cfg/vcam.cfg
  )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${VCAM_LIB_NAME} ${VCAM_NODELET_NAME} vcam
 CATKIN_DEPENDS roscpp nodelet dynamic_reconfigure cv_bridge image_transport image_geometry sensor_msgs camera_calibration_parsers camera_info_manager mavros_msgs std_srvs std_msgs message_runtime
 DEPENDS OpenCV
)

###########
## Build ##
###########

####################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${V4L_INCLUDE_DIRS}
  ${UDEV_INCLUDE_DIRS}
  )


add_library(${VCAM_LIB_NAME}  src/device.cpp src/v4l2_cam_driver.cpp include/vcam/v4l2_cam_driver.h include/vcam/device.h)
SET_TARGET_PROPERTIES(${VCAM_LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(${VCAM_LIB_NAME} ${V4L_LIBRARIES} ${UDEV_LIBRARIES} ${catkin_LIBRARIES})



add_library(${VCAM_NODELET_NAME} ${VCAM_NODELET_SOURCES} ${VCAM_NODELET_HEADERS})



target_link_libraries(${VCAM_NODELET_NAME} ${VCAM_LIB_NAME} ${Boost_INCLUDE_DIRS} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBRARIES} ${libudev_LIBRARIES} ${catkin_LIBRARIES} )
add_dependencies(${VCAM_NODELET_NAME} ${PROJECT_NAME}_gencfg)


# target_link_libraries(${PROJECT_NAME}_nodelet
#   vcam
#   ${VCAM_LIB_NAME}
#   ${catkin_LIBRARIES}
#   ${libudev_LIBRARIES}
# )


# add_library(${PROJECT_NAME}
#   src/device.cpp
#   src/cam_driver.cpp
# )

add_executable(trigger_commander
  src/TriggerSwitch.cpp
)
target_link_libraries(trigger_commander
  ${catkin_LIBRARIES}
)


#################################################


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   ${libudev_LIBRARIES}
# )

#############
## Install ##
#############





install(TARGETS ${VCAM_LIB_NAME} ${VCAM_NODELET_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )




# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vcam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
