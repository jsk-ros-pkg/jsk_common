project(nlopt)

cmake_minimum_required(VERSION 2.4.6)

execute_process(COMMAND rosmake nlopt)
message("rosmake nlopt")
