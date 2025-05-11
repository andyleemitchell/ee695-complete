cmake_minimum_required(VERSION 3.16...3.31)

project(c4sim 
    VERSION 0.1
    DESCRIPTION "Simulator for Connect Four Solvers"
    LANGUAGES CXX
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(raylib)

# put output binary into /build/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# put output binary into ./bin
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

add_subdirectory(apps)

# add this to include any libraries
add_subdirectory(src)


