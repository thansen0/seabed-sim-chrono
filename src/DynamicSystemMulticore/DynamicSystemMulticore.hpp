#pragma once

#include <iostream>
#include <toml++/toml.h>

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/collision/ChCollisionSystem.h"

#include "chrono_vehicle/terrain/GranularTerrain.h"
#include "chrono/physics/ChBodyEasy.h"

enum class TerrainType {
    RIGID,
    DEM
};

class DynamicSystemMulticore {
private:
    // really should be global in a namespace with TerrainType
    constexpr static double gravitational_const = -9.81; // m/s^2

    TerrainType terrain_type;
    chrono::ChSystemMulticore *sys;
    chrono::vehicle::GranularTerrain *terrain;
    std::shared_ptr<chrono::ChBodyEasyBox> ground; // TODO I don't like how these are two things
    std::shared_ptr<chrono::ChContactMaterial> mat;

    // default values, consider removing if we want to force
    // config file setup
    double particle_r   = 0.006;    // DEM particle radius (meters)
    double particle_rho = 2000.0;   // particle density (kg/m^3)
    uint32_t layers     = 3;        // number of initial layers

    /* Must be called during one of the constructors, otherwise
     * the system will not be set up properly
     */
    void InitializeSystem();

public:
    explicit DynamicSystemMulticore(TerrainType);
    DynamicSystemMulticore(TerrainType, toml::table&);
    ~DynamicSystemMulticore();

    std::shared_ptr<chrono::ChContactMaterial> GetMat();
    chrono::ChSystemMulticore* GetSys();

    void GenerateTerrain(double, double);

    void AdvanceAll(double);

    void Add(std::shared_ptr<chrono::ChBody>);
};
