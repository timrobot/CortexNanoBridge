cmake_minimum_required(VERSION 3.10)
project(CortexNanoBridge)

find_package(Threads REQUIRED)

add_definitions("-std=c++11 -O2")

add_executable(main main.c vex_comms.c)
target_link_libraries(main Threads::Threads)
