cmake_minimum_required(VERSION 2.8.3)
project(pathMotionAction)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  std_msgs
  roslib
  message_generation
)

find_package(Eigen3 REQUIRED)

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action
   FILES
   PurePursuit.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
 )

catkin_package(CATKIN_DEPENDS actionlib_msgs)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
include_directories("/usr/include/eigen3")

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
## Declare a C++ executable

add_executable(server_node 
src/server.cpp
src/moveP2PServer.cpp
src/purepursitFunction.cpp)

add_executable(client1 
src/client1.cpp
src/moveP2PClient.cpp)
add_executable(client2 
src/client2.cpp
src/moveP2PClient.cpp)

## Add cmake target dependencies of the executable
## same as for the library above

add_dependencies(server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(client1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(client2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(client1 ${catkin_LIBRARIES})
target_link_libraries(client2 ${catkin_LIBRARIES})

 target_link_libraries(server_node
   ${catkin_LIBRARIES} libCmpAgu.a
 )

 