cmake_minimum_required(VERSION 3.2)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

set(lib_name "matchinglib_poselib_ros")
project(${lib_name})

set(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_PREFIX_PATH})

find_package(OpenCV 4.2.0 REQUIRED core highgui imgproc features2d calib3d video)

find_package(matchinglib_poselib REQUIRED)

find_package(Eigen REQUIRED)

find_package(Clapack REQUIRED)
#if (CLAPACK_FOUND STREQUAL "YES")
#  message(STATUS "Clapack found!")
#else()
#  message(FATAL_ERROR "Clapack not found!")
#endif()

find_package(SBA REQUIRED)
#if (SBA_FOUND STREQUAL "YES")
#  message(STATUS "SBA found!")
#else()
#  message(FATAL_ERROR "SBA not found!")
#endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
SET(ROS_VERSION "indigo" CACHE STRING "select which version of ROS is installed")
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/${ROS_VERSION}")
find_package(catkin REQUIRED COMPONENTS
  ait_common
  cmake_modules
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  message_filters
  sensor_msgs
  std_msgs
)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
#add_message_files(
#    FILES
#    State.msg
#   Message2.msg
#)

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
#generate_messages(
#   DEPENDENCIES
#    std_msgs
#   obstacle_msgs#   sensor_msgs#   std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/dyn_config.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES ${lib_name}
  CATKIN_DEPENDS
    ait_common
    cv_bridge
    dynamic_reconfigure
    geometry_msgs
    sensor_msgs
  DEPENDS
    OpenCV
    system_lib
)

###########
## Build ##
###########
set(ROS_SOURCES_FILES
  "include/ROSnode/ROSdynamic.hpp"
  "include/ROSnode/ROSnode.hpp"
  "include/ROSnode/BufferData.hpp"
  "src/ROSnode.cpp"
  "src/BufferData.cpp"
)

set(PRJ_SOURCE_FILES
  ${ROS_SOURCES_FILES}
  # here goes the rest!
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -std=c++11 -Wall -Wno-reorder -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O2")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
   include
   ${DEFAULT_INCLUDE_DIRECTORIES}
   ${catkin_INCLUDE_DIRS}
   ${Eigen_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS}
   ${system_lib_INCLUDE_DIRS}
   matchinglib_poselib::matchinglib
   matchinglib_poselib::poselib
        ${CLAPACK_INCLUDE_DIRS}
)

####
## Declare a C++ library
add_library(${lib_name} STATIC
 ${PRJ_SOURCE_FILES}
)

#LINK_DIRECTORIES(/usr/local/lib)

target_link_libraries(${lib_name}
  PUBLIC

  ${catkin_LIBRARIES}
        sba
        ${CLAPACK_LIBRARIES}
  matchinglib_poselib::matchinglib
  matchinglib_poselib::poselib
  ${OpenCV_LIBS}
  ${system_lib_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${lib_name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

####
## Declare a C++ executable
set(binary_name "${lib_name}_node")

add_executable(${binary_name} src/main.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${binary_name}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(${binary_name}
  ${lib_name}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${binary_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
install(FILES
  launch/default_config.yaml
  launch/default.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

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
catkin_add_gtest(${PROJECT_NAME}-test test/test.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
