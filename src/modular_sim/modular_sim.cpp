#include <chrono> // different chrono...
#include <memory>
#include <random>
#include <iostream>

#include "DynamicSystemMulticore.hpp"

#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/collision/ChCollisionSystem.h"

#include "chrono_vehicle/terrain/GranularTerrain.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vehicle;

/* / Patch parameters
constexpr double patch_length = 1.5;     // X size
constexpr double patch_width  = 1.5;     // Y size
constexpr double particle_r   = 0.005;     // DEM particle radius (meters)
constexpr double particle_rho = 2000.0;   // particle density (kg/m^3)
constexpr unsigned int layers = 6;        // number of initial layers
*/
int main() {
    // Random placement for the rigid balls
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-patch_length/2, patch_length/2);

    std::uniform_real_distribution<double> radius_dist(0.01, 0.04);

    chrono::SetChronoDataPath("/home/thomas/Code/seabed_sim/chrono/data/");

    // ---------------------------------------------------------
    // 1) Physics system: MULTICORE + SMC (required for DEM style)
    // ---------------------------------------------------------

    DynamicSystemMulticore sys(TerrainType::DEM);
    // DynamicSystemMulticore sys(TerrainType::RIGID);

    // ---------------------------------------------------------
    // 2) Generate Terrain (based on TerrainType)
    // ---------------------------------------------------------
    sys.GenerateTerrain(patch_length, patch_width);

    // -----------------------------------------
    // 3) Rigid spheres
    // -----------------------------------------
    for (int i = 0; i < 2; i++) {
        std::shared_ptr<ChBodyEasySphere> ball = chrono_types::make_shared<ChBodyEasySphere>(
            radius_dist(gen),     // radius
            1000.0,   // density
            true,     // visual
            true,     // collision
            sys.GetMat() // mat
        );

        ball->SetPos(ChVector3d(dist(gen), dist(gen), 1.0));
        ball->EnableCollision(true);

        // add color to make it easier to see
        auto vis_shape = ball->GetVisualShape(0);
        auto mat = chrono_types::make_shared<ChVisualMaterial>();

        mat->SetDiffuseColor(ChColor(0.8f, 0.1f, 0.1f)); // red
        vis_shape->SetMaterial(0, mat);

        sys.Add(ball);
    }

    // -----------------------------------------
    // 4) Visualization (VSG)
    // -----------------------------------------
    auto vis = chrono_types::make_shared<chrono::vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(sys.GetSys());

    vis->SetWindowTitle("Chrono 9: Multicore SMC + GranularTerrain (DEM)");
    vis->SetWindowSize(1280, 720);
    vis->SetClearColor(ChColor(0.1f, 0.1f, 0.12f));
    vis->AddCamera(ChVector3d(0, -25, 12), ChVector3d(0, 0, 0));
    vis->SetLightIntensity(1.5f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);

    auto start = std::chrono::high_resolution_clock::now();
    vis->Initialize();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Viz init in " << duration << std::endl;

    // -----------------------------------------
    // 5) Sim loop
    // -----------------------------------------
    const double step = 1e-3;
    ChRealtimeStepTimer realtime;

    while (vis->Run()) {
        // goal is to only render a frame every couple of
        // simulation iterations
        for (int i = 0; i < 5; i++) {
            sys.AdvanceAll(step);
        }

        auto start = std::chrono::high_resolution_clock::now();

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "VIS took " << duration << std::endl;

        realtime.Spin(step);
    }

    return 0;
}

