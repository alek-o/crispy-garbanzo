cmake_minimum_required(VERSION 3.10)
project(BTSP)

include_directories(include)

set(SOURCES
    src/main.cpp
    src/Graph.cpp
    src/GraphGenerator.cpp
)

add_executable(my_program ${SOURCES})
