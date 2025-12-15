add_executable(dem_sim dem_terrain.cpp)

# Pull in shared deps/flags/includes
target_link_libraries(dem_sim PRIVATE sim_common)
