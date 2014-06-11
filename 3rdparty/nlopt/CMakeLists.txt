project(nlopt)

cmake_minimum_required(VERSION 2.4.6)

message("cmake -E chdir ${PROJECT_SOURCE_DIR}/.. make -f ${PROJECT_SOURCE_DIR}/Makefile")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f ${PROJECT_SOURCE_DIR}/Makefile)
