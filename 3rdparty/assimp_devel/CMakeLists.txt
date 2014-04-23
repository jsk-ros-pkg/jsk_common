# for catkin only
cmake_minimum_required(VERSION 2.8.3)
project(assimp_devel)
find_package(catkin REQUIRED COMPONENTS mk)

add_custom_target(libassimp_devel ALL
  DEPENDS ${CATKIN_DEVEL_PREFIX}/lib/libassimp_devel.so)
add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/lib/libassimp_devel.so
  DEPENDS Makefile
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR} make -f ${PROJECT_SOURCE_DIR}/Makefile MK_DIR=${mk_PREFIX}/share/mk INSTALL_DIR=${CATKIN_DEVEL_PREFIX} PATCH_DIR=${PROJECT_SOURCE_DIR})

# fake install directory for catkin_package
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/assimp_devel)
if("$ENV{ROS_DISTRO}" STREQUAL "groovy")
  catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/assimp_devel
    LIBRARIES    assimp_devel
    SKIP_CMAKE_CONFIG_GENERATION
    )
  set(${PROJECT_NAME}_EXPORTED_TARGETS libassimp_devel)
else()
  catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/assimp_devel
    LIBRARIES    assimp_devel
    EXPORTED_TARGETS libassimp_devel
    SKIP_CMAKE_CONFIG_GENERATION
    )
endif()
execute_process(COMMAND sed -i s@${${PROJECT_NAME}_VERSION}@3.0.1@ ${CATKIN_DEVEL_PREFIX}/lib/pkgconfig/assimp_devel.pc)

install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/bin/assimp_devel
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(CODE "
  file(GLOB _libs ${CATKIN_DEVEL_PREFIX}/lib/libassimp_devel.*)
  foreach(_lib \${_libs})
    FILE(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/lib\" TYPE PROGRAM FILES \"\${_lib}\" )
  endforeach()
")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/assimp_devel
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES ${CATKIN_DEVEL_PREFIX}/lib/pkgconfig/assimp_devel.pc
        DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}/pkgconfig)
install(CODE "execute_process(COMMAND sed -i s@${CATKIN_DEVEL_PREFIX}@${CMAKE_INSTALL_PREFIX}@g \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/pkgconfig/assimp_devel.pc)")

# only for deb packaging...
if ($ENV{DH_OPTIONS}) # deb package need to remove generated files
  install(CODE "execute_process(COMMAND cmake -E remove_directory ${CATKIN_DEVEL_PREFIX}")
endif()
