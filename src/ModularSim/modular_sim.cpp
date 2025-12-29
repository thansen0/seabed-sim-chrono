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

std::string config_path = "../config/config.toml";

double sim_length;                          // X size
double sim_width;                           // Y size
constexpr double sim_particle_height{0.5};  // Z pos
double sim_step_size{1e-3};
int steps_per_frame{10};

int main(int argc, char* argv[]) {
    TerrainType terrain_type = TerrainType::DEM;
    chrono::SetChronoDataPath("/home/thomas/Code/seabed_sim/chrono/data/");

    // ---------------------------------------------------------
    // Configure Runtime Arguments
    // ---------------------------------------------------------
    if (argc > 1) {
        unsigned int cur_arg = 1;

        while (cur_arg < static_cast<unsigned int>(argc)) {
            std::string arg1 = argv[1];

            trim_chars(arg1, "-");
            lower(arg1);
            if (arg1 == "rigid") {
                terrain_type = TerrainType::RIGID;
            } else if (arg1 == "dem") {
                terrain_type = TerrainType::DEM;
            } else if (arg1 == "config") {
                if (cur_arg + 1 < static_cast<unsigned int>(argc)) {
                    config_path = argv[++cur_arg];
                    std::cout << "Updated config path to " << config_path << std::endl;
                } else {
                    std::cout << "No config path provided after --config" << std::endl;
                    return 1;
                }
            } else {
                std::cout << "Unknown terrain type argument: " << arg1 << std::endl;
                std::cout << "Valid options are: --rigid, --dem, --config \"path/to/config.toml\"\n";
                return 1;
            }

            cur_arg++;
        }
    }

    // ---------------------------------------------------------
    // Build config file
    // ---------------------------------------------------------
    toml::table config_tbl = parse_toml_file(config_path);

    // ---------------------------------------------------------
    // Physics System Manager
    // ---------------------------------------------------------
    DynamicSystemMulticore sys(terrain_type, config_tbl);

    // ---------------------------------------------------------
    // Generate Terrain (based on TerrainType)
    // ---------------------------------------------------------
    sys.GenerateTerrain(sim_length, sim_width);

    // -----------------------------------------
    // Create Nodules
    // -----------------------------------------
    PatchLogNormalNodules generator(config_tbl, &sys);
    auto nodules = generator.generate_nodules();

    auto start = std::chrono::high_resolution_clock::now();
    for (const auto& n : nodules) {
        std::shared_ptr<ChBody> ball = n.nodule;

        ball->SetPos(ChVector3d(n.x - (sim_length / 2.0), n.y - (sim_width / 2.0), sim_particle_height));
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
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << nodules.size() << " nodles generated in " << duration << std::endl;

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

    start = std::chrono::high_resolution_clock::now();
    vis->Initialize();
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Viz init in " << duration << std::endl;

    // -----------------------------------------
    // Main loop
    // -----------------------------------------
    ChRealtimeStepTimer realtime;

    while (vis->Run()) {
        // -----------------------------------------
        // Advance Simulation
        // -----------------------------------------
        auto start = std::chrono::high_resolution_clock::now();
        // goal is to only render a frame every couple of
        // simulation iterations
        for (int i = 0; i < steps_per_frame; i++) {
            sys.AdvanceAll(sim_step_size);
        }
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "VIS took " << duration << ", or for each itr: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration / steps_per_frame) << std::endl;

        // -----------------------------------------
        // Scene rendering
        // -----------------------------------------
        start = std::chrono::high_resolution_clock::now();

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "VIS took " << duration << std::endl;

        realtime.Spin(sim_step_size);
    }

    return 0;
}

