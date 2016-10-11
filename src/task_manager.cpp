cmake_minimum_required(VERSION 2.8.3)
project(task_assign)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)


find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)

add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

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
 add_message_files(
   FILES
   AgentStatus.msg
   SecondAgent.msg
   IniStatus.msg
   AssignMsg.msg
   OneAssign.msg
   task.msg
   vect_task.msg
   robot.msg
   vect_robot.msg
   rt.msg
   rt_vect.msg
   info.msg
   vect_info.msg
   waypoint.msg
   task_path.msg
   assignment.msg
   recharge.msg
   rech_vect.msg
   glpk_in.msg
   glpk_sol.msg
 )

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
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

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
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES task_assign
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime yaml-cpp lemon_ros gmlreader
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(task_assign
#   src/${PROJECT_NAME}/task_assign.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(task_assign ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(task_assign_node src/task_assign_node.cpp)
add_executable(robot_node src/robot_node.cpp)
add_executable(task_node src/task_node.cpp)
add_executable(central_node src/central_node.cpp)
# add_executable(users_node src/users_node.cpp)
add_executable(task_manager src/task_manager.cpp)
add_executable(motion_planner src/motion_planner.cpp)
add_executable(obstacle_node src/obstacle_node.cpp)
add_executable(master src/master.cpp)
add_executable(robot src/robot.cpp)
# add_executable(temp src/temp.cpp)
# add_executable(map src/map.cpp)
add_executable(monsters_test src/monsters_test.cpp)
# add_executable(task_test_config src/task_test_config.cpp)
# add_executable(robot_node src/robot_node_2.cpp)
# add_executable(task_node src/task_node_2.cpp)
# add_executable(central_node src/central_node_2.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(task_assign_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_node ${PROJECT_NAME}_gencpp)
add_dependencies(task_node ${PROJECT_NAME}_gencpp)
add_dependencies(central_node ${PROJECT_NAME}_gencpp)
# add_dependencies(users_node ${PROJECT_NAME}_gencpp)
add_dependencies(task_manager ${PROJECT_NAME}_gencpp)
add_dependencies(motion_planner ${PROJECT_NAME}_gencpp)
add_dependencies(obstacle_node ${PROJECT_NAME}_gencpp)
add_dependencies(master ${PROJECT_NAME}_gencpp)
add_dependencies(robot ${PROJECT_NAME}_gencpp)
# add_dependencies(temp ${PROJECT_NAME}_gencpp)
# add_dependencies(reass_manager ${PROJECT_NAME}_gencpp)
# add_dependencies(map ${PROJECT_NAME}_gencpp)
add_dependencies(monsters_test ${PROJECT_NAME}_gencpp)
# add_dependencies(task_test_config ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(task_assign_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(robot_node ${catkin_LIBRARIES})
target_link_libraries(task_node ${catkin_LIBRARIES})
target_link_libraries(central_node ${catkin_LIBRARIES})
# target_link_libraries(users_node yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(task_manager yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(motion_planner lemon_ros gmlreader yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(obstacle_node ${catkin_LIBRARIES})
target_link_libraries(master glpk yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(robot ${catkin_LIBRARIES})
# target_link_libraries(temp yaml-cpp ${catkin_LIBRARIES})
# target_link_libraries(reass_manager ${catkin_LIBRARIES})
# target_link_libraries(map ${catkin_LIBRARIES})
target_link_libraries(monsters_test yaml-cpp ${catkin_LIBRARIES})
# target_link_libraries(task_test_config ${catkin_LIBRARIES})


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
# install(TARGETS task_assign task_assign_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_task_assign.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_definitions(-std=gnu++11)
