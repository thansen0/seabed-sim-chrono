#include <iostream>
#include "DynamicSystemMulticore.hpp"
#include "chrono/physics/ChSystem.h"

using namespace chrono;

DynamicSystemMulticore::DynamicSystemMulticore(TerrainType tt)
    : terrain_type{tt}
{
    switch (tt) {
        case TerrainType::RIGID:
            this->sys = new ChSystemMulticoreNSC();

            sys->SetNumThreads(std::thread::hardware_concurrency());
            sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

            // pick Bullet collision
            sys->SetCollisionSystemType(chrono::ChCollisionSystem::Type::MULTICORE);

            mat = chrono_types::make_shared<ChContactMaterialNSC>();
            mat->SetFriction(0.6f);
            mat->SetRestitution(0.1f);

            break;
        case TerrainType::DEM:
            this->sys = new ChSystemMulticoreSMC();

            this->sys->SetNumThreads(std::thread::hardware_concurrency());
            this->sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

            // Multicore collision (pairs well with ChSystemMulticore*)
            this->sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

            // Contact material MUST match the system contact method (SMC here)
            mat = chrono_types::make_shared<ChContactMaterialSMC>();
            mat->SetFriction(0.6f);
            mat->SetRestitution(0.1f);

            break;
    }
}

DynamicSystemMulticore::~DynamicSystemMulticore() {
    delete this->sys;
    delete this->terrain;
}

void DynamicSystemMulticore::GenerateTerrain(double length, double width)
{
    switch (this->terrain_type) {
        case TerrainType::RIGID: {
            std::cout << "Rigid terrain" << std::endl;
            chrono::ChSystemMulticoreNSC *smc_sys = static_cast<chrono::ChSystemMulticoreNSC*>(this->sys);

            this->ground = chrono_types::make_shared<chrono::ChBodyEasyBox>(
                length, width, 1.0,   // size (x,y,z)
                1000.0,            // density (irrelevant since fixed)
                true,              // visual shape
                true,              // collision shape
                mat
            );
            ground->SetFixed(true);
            ground->SetPos(ChVector3d(0, 0, -0.5));  // top surface at z=0
            ground->EnableCollision(true);
            smc_sys->Add(ground);

            break;
        }
        case TerrainType::DEM: {
            std::cout << "DEM terrain" << std::endl;
            ChSystemMulticoreSMC *smc_sys = static_cast<ChSystemMulticoreSMC*>(this->sys);

            terrain = new chrono::vehicle::GranularTerrain(this->sys);
            terrain->SetContactMaterial(mat);

            // add fixed “roughness” spheres at the bottom to reduce bed sliding
            terrain->EnableRoughSurface(40, 40);

            // shows the container boundaries (not the particles)
            terrain->EnableVisualization(true);

            auto start = std::chrono::high_resolution_clock::now();
            // Initialize: center is the *center of the bottom* of the patch :contentReference[oaicite:1]{index=1}
            terrain->Initialize(ChVector3d(0, 0, 0), length, width, layers, particle_r, particle_rho);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            std::cout << "DEM initialized in " << duration << std::endl;

            break;
        }
        default:
            std::cout << "Error! Probably return something naughty" << std::endl;
            break;
    
    }

}


void DynamicSystemMulticore::AdvanceAll(double step) {
    switch (this->terrain_type) {
        case TerrainType::RIGID:{
            // Advance dynamics
            sys->DoStepDynamics(step);
            break;
        }
        case TerrainType::DEM: {
            ChSystemMulticoreSMC *smc_sys = static_cast<ChSystemMulticoreSMC*>(this->sys);

            double t = smc_sys->GetChTime();
            terrain->Synchronize(t);
            terrain->Advance(step);

            smc_sys->DoStepDynamics(step);

            break;
        }
        default:
            break;
    }
}

std::shared_ptr<chrono::ChContactMaterial> DynamicSystemMulticore::GetMat() {
    return this->mat;
}

chrono::ChSystemMulticore* DynamicSystemMulticore::GetSys() {
    return this->sys;
}

void DynamicSystemMulticore::Add(std::shared_ptr<chrono::ChBody> obj) {
    ChSystemMulticoreSMC *smc_sys = static_cast<ChSystemMulticoreSMC*>(this->sys);
    smc_sys->Add(obj);
}

