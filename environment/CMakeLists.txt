cmake_minimum_required(VERSION 2.8.3)
project(environment)

set(PKG_DEPENDS
  )
set(BUILD_DEPENDS
  ${PKG_DEPENDS}
  )
set(RUN_DEPENDS
  ${PKG_DEPENDS}
  )

#######################################
## Find dependencies
#######################################

find_package(catkin REQUIRED ${BUILD_DEPENDS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

#######################################
## Declare Catkin package and installation
#######################################

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
  )

install(DIRECTORY unity
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  )


install(DIRECTORY overhead_mesh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  )






