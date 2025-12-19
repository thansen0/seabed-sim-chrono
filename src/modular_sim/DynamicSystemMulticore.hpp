#pragma once

#include <iostream>

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/collision/ChCollisionSystem.h"

#include "chrono_vehicle/terrain/GranularTerrain.h"
#include "chrono/physics/ChBodyEasy.h"

enum class TerrainType {
    RIGID,
    DEM
};


constexpr double patch_length = 1.5;     // X size
constexpr double patch_width  = 1.5;     // Y size
constexpr double particle_r   = 0.005;     // DEM particle radius (meters)
constexpr double particle_rho = 2000.0;   // particle density (kg/m^3)
constexpr unsigned int layers = 6;        // number of initial layers

class DynamicSystemMulticore { // : public chrono::ChSystemMulticore {
private:
    TerrainType terrain_type;
    // std::unique_ptr<chrono::ChSystemMulticore> sys;
    chrono::ChSystemMulticore *sys;
    chrono::vehicle::GranularTerrain *terrain;
    std::shared_ptr<chrono::ChContactMaterialSMC> mat;

public:
    explicit DynamicSystemMulticore(TerrainType);
    ~DynamicSystemMulticore();

    std::shared_ptr<chrono::ChContactMaterialSMC> GetMat();
    chrono::ChSystemMulticore* GetSys();

    void GenerateTerrain(double, double);

    void AdvanceAll(double step);

    // void Add(std::shared_ptr<ChBodyEasySphere> obj);
    void Add(std::shared_ptr<chrono::ChBody> obj);
};
