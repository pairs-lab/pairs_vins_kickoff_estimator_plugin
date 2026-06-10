cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(pairs_vins_kickoff_estimator_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pairs_uav_managers
  pairs_uav_state_estimators
  )

set(LIBRARIES
  PairsUavStateEstimators_VinsKickoff
  )

catkin_package(
  CATKIN_DEPENDS
  pairs_uav_managers
  pairs_uav_state_estimators
  )


include_directories(
  include
  )

find_package(pairs_uav_state_estimators REQUIRED)

###########
## Build ##
###########

add_library(PairsUavStateEstimators_VinsKickoff
  src/vins_kickoff_estimator_plugin.cpp
  )

add_dependencies(PairsUavStateEstimators_VinsKickoff
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavStateEstimators_VinsKickoff
  ${catkin_LIBRARIES}
  )

target_include_directories(PairsUavStateEstimators_VinsKickoff
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY tmux/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES estimator_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
