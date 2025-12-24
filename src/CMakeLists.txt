add_executable(
    modular_sim
    ModularSim/modular_sim.cpp
    DynamicSystemMulticore/DynamicSystemMulticore.cpp
    ModularSim/HelperFunctions.cpp
    NodeGen/PatchLogNormalNodules.cpp
)

include_directories(DynamicSystemMulticore/)
include_directories(ModularSim/)
include_directories(NodeGen/)

# Pull in shared deps/flags/includes
target_link_libraries(modular_sim PRIVATE sim_common tomlplusplus::tomlplusplus)
