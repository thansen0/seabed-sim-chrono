add_executable(modular_sim modular_sim.cpp dynamic_system_multicore.cpp helper_functions.cpp PatchLogNormalNodules.cpp)

# Pull in shared deps/flags/includes
target_link_libraries(modular_sim PRIVATE sim_common)
