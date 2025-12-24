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

#include "HelperFunctions.hpp"
#include "PatchLogNormalNodules.hpp"

using namespace chrono;
using namespace chrono::vehicle;



int main(int argc, char* argv[]) {
    TerrainType terrain_type = TerrainType::DEM;

    // Random placement for the rigid balls
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-patch_length/2, patch_length/2);

    // std::uniform_real_distribution<double> radius_dist(0.01, 0.04);
    std::normal_distribution<double> radius_dist(.002185, 0.007737);

    chrono::SetChronoDataPath("/home/thomas/Code/seabed_sim/chrono/data/");

    // ---------------------------------------------------------
    // Configure Runtime Arguments
    // ---------------------------------------------------------
    if (argc > 1) {
        std::string arg1 = argv[1];
        trim_chars(arg1, "-");
        lower(arg1);
        if (arg1 == "rigid") {
            terrain_type = TerrainType::RIGID;
        } else if (arg1 == "dem") {
            terrain_type = TerrainType::DEM;
        } else {
            std::cout << "Unknown terrain type argument: " << arg1 << "\n";
            std::cout << "Valid options are: --rigid, --dem\n";
            return 1;
        }
    }

    // ---------------------------------------------------------
    // Physics System Manager
    // ---------------------------------------------------------
    DynamicSystemMulticore sys(terrain_type);

    // ---------------------------------------------------------
    // Generate Terrain (based on TerrainType)
    // ---------------------------------------------------------
    sys.GenerateTerrain(patch_length, patch_width);

    // -----------------------------------------
    // Create Nodules
    // -----------------------------------------
    PatchLotNormalNodules generator("future_config_path", &sys);
    auto nodules = generator.generate_nodules();

    std::cerr << "Generated " << nodules.size() << " nodules\n";
    for (const auto& n : nodules) {
        std::shared_ptr<ChBody> ball = n.nodule;

        ball->SetPos(ChVector3d(n.x - (patch_length / 2.0), n.y - (patch_width / 2.0), 1.0));
        ball->EnableCollision(true);

        // add color to make it easier to see
        auto vis_shape = ball->GetVisualShape(0);

        // set system contact material
        // ball->GetCollisionModel()->SetAllShapesMaterial(sys.GetMat());

        // sets visual color material
        auto mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetDiffuseColor(ChColor(0.8f, 0.1f, 0.1f)); // red
        vis_shape->SetMaterial(0, mat);

        sys.Add(ball);
    }

    // -----------------------------------------
    // Visualization with VSG
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
    // Main loop
    // -----------------------------------------
    const double step = 1e-3;
    const int steps_per_frame = 10;
    ChRealtimeStepTimer realtime;

    while (vis->Run()) {
        // goal is to only render a frame every couple of
        // simulation iterations
        for (int i = 0; i < steps_per_frame; i++) {
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

