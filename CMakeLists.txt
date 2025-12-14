cmake_minimum_required(VERSION 3.16)
project(chrono_minimal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake exactly where chrono-config.cmake lives:
# <chrono_build>/cmake/chrono-config.cmake
set(Chrono_DIR "/home/thomas/Code/seabed_sim/chrono/build/cmake")

list(PREPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Packages/vsg")

find_package(Chrono REQUIRED COMPONENTS VSG)
find_package(vsg REQUIRED)

add_executable(chrono_minimal main.cpp)

# expose headers
target_include_directories(chrono_minimal PRIVATE
    ${CHRONO_INCLUDE_DIRS}
    /usr/include/irrlicht
    /home/thomas/Packages/vsg/include/vsg
)

# Link against whatever Chrono exported for your build
target_link_libraries(chrono_minimal PRIVATE
    ${CHRONO_LIBRARIES}
    #ChronoEngine
    #ChronoEngine_vsg
    #ChronoModels_robot
    #Chrono::ChronoEngine
    #Chrono::ChronoEngine_vsg
    #Chrono::ChronoModels_robot
    Irrlicht
    vsg::vsg
    # ${CHRONO_TARGETS}
    # Chrono::chrono
    # Chrono::chrono_vehicle
    # Chrono::chrono_irrlicht
)

message(STATUS "CHRONO_INCLUDE_DIRS = ${CHRONO_INCLUDE_DIRS}")
message(STATUS "CHRONO_LIBRARIES    = ${CHRONO_LIBRARIES}")
message(STATUS "CHRONO_TARGETS      = ${CHRONO_TARGETS}")
