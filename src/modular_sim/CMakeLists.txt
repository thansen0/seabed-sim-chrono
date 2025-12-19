add_executable(modular_sim modular_sim.cpp dynamic_system_multicore.cpp)

# Pull in shared deps/flags/includes
target_link_libraries(modular_sim PRIVATE sim_common)
