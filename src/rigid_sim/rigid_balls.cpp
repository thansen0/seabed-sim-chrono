#include <memory>
#include <random>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono_multicore/physics/ChSystemMulticore.h" // ChSystemMulticoreNSC
#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono/collision/ChCollisionSystem.h"

using namespace chrono;

int main() {
    // pseudo random number generator
    std::random_device rd;
    std::mt19937 gen(rd());  // Mersenne Twister
    std::uniform_real_distribution<double> dist(-50.0, 50.0);

    // set data path
    // vsg/share/vsgExamples/textures/
    chrono::SetChronoDataPath("/home/thomas/Code/seabed_sim/chrono/data/");

    // -----------------------------
    // 1) Physics system
    // -----------------------------
    // ChSystemNSC sys;
    ChSystemMulticoreNSC sys;
    sys.SetNumThreads(std::thread::hardware_concurrency());
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // pick Bullet collision
    sys.SetCollisionSystemType(chrono::ChCollisionSystem::Type::MULTICORE);
    
    // add solver 
    //sys.SetSolverType(ChSolver::Type::PSOR);
    //sys.GetSolver()->AsIterative()->SetMaxIterations(50);
    //auto settings = sys.GetSettings();
    //settings->solver.max_iteration = 50;      // or settings->solver.max_iter
    //settings->solver.tolerance = 1e-4;        // optional

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.6f);
    mat->SetRestitution(0.1f);

    // -----------------------------
    // 2) Rigid terrain (fixed ground)
    // -----------------------------
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(
        100.0, 100.0, 1.0,   // size (x,y,z)
        1000.0,            // density (irrelevant since fixed)
        true,              // visual shape
        true,              // collision shape
        mat
    );
    ground->SetFixed(true);
    ground->SetPos(ChVector3d(0, 0, -0.5));  // top surface at z=0
    ground->EnableCollision(true);
    sys.Add(ground);

    // Optional: set some contact material properties (simple)
    // (Chrono will use defaults if you skip this)

    // -----------------------------
    // 3) A falling object to see motion
    // -----------------------------
    for (int i = 0; i < 300; i++) {
        auto ball = chrono_types::make_shared<ChBodyEasySphere>(
            0.35,              // radius
            1000.0,            // density
            true,              // visual
            true,              // collision
            mat
        );
        
        ball->SetPos(ChVector3d(dist(gen), dist(gen), 2.5));
        ball->EnableCollision(true);
        sys.Add(ball);
    }

    // -----------------------------
    // 4) Visualization system (VSG)
    // -----------------------------
    auto vis = chrono_types::make_shared<chrono::vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);

    // Window + scene basics
    vis->SetWindowTitle("Chrono 9: Rigid Terrain (VSG)");
    vis->SetWindowSize(1280, 720);
    vis->SetClearColor(ChColor(0.1f, 0.1f, 0.12f));

    // Camera setup: eye, target, up
    vis->AddCamera(ChVector3d(0, -12, 6), ChVector3d(0, 0, 0)); // , ChVector3d(0, 0, 1));
    // vis->SetCameraVertical(chrono::vsg3d::ChVisualSystemVSG::CameraVerticalDir::Z);

    // Lights
    vis->SetLightIntensity(1.5f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);

    vis->Initialize();

    auto coll_sys = sys.GetCollisionSystem();

    if (!coll_sys) {
        std::cout << "No collision system attached!\n";
    } else {
        std::cout << "Collision system class name: "
                  << typeid(*coll_sys).name() << "\n";
    }
    
    // -----------------------------
    // 5) Sim loop
    // -----------------------------
    const double step = 1e-3;
    ChRealtimeStepTimer realtime;

    while (vis->Run()) {
        // Advance dynamics
        sys.DoStepDynamics(step);

        // Render
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Optional real-time pacing
        realtime.Spin(step);
    }

    return 0;
}

