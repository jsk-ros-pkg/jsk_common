cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Generate the file foo
# Create a target that depends on the generation of foo
#add_custom_command(
#  OUTPUT  include/AquesTalk2.h
#  COMMAND mkdir -p include && cp $(SOURCE_DIR)/lib64/*.h include
#  )
#add_custom_target(include_target DEPENDS include/AquesTalk2.h)
#add_dependencies(rosbuild_precompile include/AquesTalk2.h)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
link_directories(${PROJECT_SOURCE_DIR}/lib)
rosbuild_add_executable(SampleTalk src/SampleTalk.c)
target_link_libraries(SampleTalk AquesTalk2)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
set(AQTK2_LNX_LIB_DIR "lib64")
else ()
set(AQTK2_LNX_LIB_DIR "lib")
endif ()

#
rosbuild_download_data(http://www.a-quest.com/download/package/aqtk2-lnx-eva_210.zip build/aqtk2-lnx-eva_210.zip 68029448ecf4d36337e754c6eb3bb5d8)
# Generate the file foo
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/include/AquesTalk2.h
  COMMAND cd ${CMAKE_CURRENT_SOURCE_DIR} && 
      unzip -u build/aqtk2-lnx-eva_210.zip -d build &&
      mkdir -p include lib phont &&
      cp build/aqtk2-lnx-eva/${AQTK2_LNX_LIB_DIR}/*.h include &&
      cp build/aqtk2-lnx-eva/${AQTK2_LNX_LIB_DIR}/libAquesTalk2Eva.so.2.1 lib/libAquesTalk2.so.2 &&
      (cd lib && ln -sf libAquesTalk2.so.2 libAquesTalk2.so ) &&
      cp -r build/aqtk2-lnx-eva/phont ./
  DEPENDS build/aqtk2-lnx-eva_210.zip
  )
# Create a target that depends on the generation of foo
add_custom_target(include_target DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/AquesTalk2.h)
# Make compilation depend on our target:
add_dependencies(SampleTalk include_target)


set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${PROJECT_SOURCE_DIR}/bin;${PROJECT_SOURCE_DIR}/include;${PROJECT_SOURCE_DIR}/lib;${PROJECT_SOURCE_DIR}/phont;${PROJECT_SOURCE_DIR}/wiped;${PROJECT_SOURCE_DIR}/installed")



