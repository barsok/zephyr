cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(audio)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

add_subdirectory(erpc)
target_link_libraries(app PRIVATE testrpc)

