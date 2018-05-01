cmake_minimum_required(VERSION 2.8.3)
project(ml_strategy_underdogs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  mg_msgs
  std_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(ml_strategy_underdogs 
               src/ml_strategy.cpp
               src/linear_algebra.cpp
               src/team_strategy_class.cpp
               src/helper.cpp
               src/threads.cpp
               src/rk4.cpp)
target_link_libraries(ml_strategy_underdogs ${catkin_LIBRARIES})
add_dependencies(ml_strategy_underdogs ${catkin_EXPORTED_TARGETS})
add_dependencies(ml_strategy_underdogs mg_msgs_generate_messages_cpp)

install(TARGETS ml_strategy_underdogs RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})