cmake_minimum_required(VERSION 2.8.3)
project(mini_maxwell)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules roslib dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/RosClient.cfg
)

#include_directories(${Boost_INCLUDE_DIRS})

# nodelet shared object
include_directories(${catkin_INCLUDE_DIRS} include)

# generate_messages()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS dynamic_reconfigure
)

macro(download_script _name _url)
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/scripts/${_name})
    execute_process(
      COMMAND wget ${_url} -O ${PROJECT_SOURCE_DIR}/scripts/${_name}
      )
  endif(NOT EXISTS ${PROJECT_SOURCE_DIR}/scripts/${_name})
endmacro(download_script _url)

download_script("mm2client.py" http://iwl.com/mmx_materials/utilities/mm2client.py)
download_script("periodic.py" http://iwl.com/mmx_materials/utilities/periodic.py)
download_script("periodic26.py" http://iwl.com/mmx_materials/utilities/periodic26.py)
download_script("setfilters.py" http://iwl.com/mmx_materials/utilities/setfilters.py)
download_script("setfilters26.py" http://iwl.com/mmx_materials/utilities/setfilters26.py)
