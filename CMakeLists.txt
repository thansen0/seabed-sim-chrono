cmake_minimum_required(VERSION 3.16)
project(chrono_minimal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake exactly where chrono-config.cmake lives:
# <chrono_build>/cmake/chrono-config.cmake
set(Chrono_DIR "$ENV{HOME}/Code/seabed_sim/chrono/build/cmake"
    CACHE PATH "Path to Chrono's CMake package folder (contains chrono-config.cmake)")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

# vsg set up using chrono script in
# chrono/contrib/build-scripts/vsg
list(PREPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Packages/vsg")

find_package(Chrono REQUIRED COMPONENTS VSG Multicore Vehicle)
find_package(vsg REQUIRED)

add_library(sim_common INTERFACE)

# expose headers
target_include_directories(sim_common INTERFACE
    ${CHRONO_INCLUDE_DIRS}
    /home/thomas/Packages/vsg/include/vsg
)

# Link against whatever Chrono exported for your build
target_link_libraries(sim_common INTERFACE
    ${CHRONO_LIBRARIES}
    vsg::vsg
)

add_subdirectory(src/rigid_sim)
# add_subdirectory(src/fea_terrain_sim)
add_subdirectory(src/dem_sim)
add_subdirectory(src/modular_sim)

message(STATUS "CHRONO_INCLUDE_DIRS = ${CHRONO_INCLUDE_DIRS}")
message(STATUS "CHRONO_LIBRARIES    = ${CHRONO_LIBRARIES}")
message(STATUS "CHRONO_TARGETS      = ${CHRONO_TARGETS}")
