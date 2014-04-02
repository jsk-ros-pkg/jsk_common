cmake_minimum_required(VERSION 2.8.3)
project(libsiftfast)

find_package(catkin)

add_custom_target(libsiftfast ALL
  DEPENDS Makefile
  COMMAND  make -f ${PROJECT_SOURCE_DIR}/Makefile INSTALL_DIR=${CATKIN_DEVEL_PREFIX})

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  LIBRARIES    siftfast
  EXPORTED_TARGETS libsiftfast
)

install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/bin/siftfast
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

file(GLOB _libs ${CATKIN_DEVEL_PREFIX}/lib/libsiftfast.*)
install(FILES ${_libs}
        DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/siftfast
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/share/siftfast
        DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)




