add_executable(rigid_balls rigid_balls.cpp)

# Pull in shared deps/flags/includes
target_link_libraries(rigid_balls PRIVATE sim_common)
