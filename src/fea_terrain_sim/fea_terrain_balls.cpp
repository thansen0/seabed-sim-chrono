#include <memory>
#include <random>

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/physics/ChSystemNSC.h"     // old can be deleted
#include "chrono_vehicle/terrain/FEATerrain.h"
#include "chrono/fea/ChMesh.h"              // for visualization
#include "chrono/assets/ChVisualShapeFEA.h" // ChVisualShapeFEA::DataType::SURFACE
#include "chrono_multicore/physics/ChSystemMulticore.h" // ChSystemMulticoreNSC
#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono/collision/ChCollisionSystem.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::vehicle;

constexpr double terrain_X{10};
constexpr double terrain_Y{10};
constexpr double H  = 0.6;      // thickness (z)

int main() {
    // pseudo random number generator
    std::random_device rd;
    std::mt19937 gen(rd());  // Mersenne Twister
    std::uniform_real_distribution<double> dist(-terrain_X/2, terrain_X/2);

    // set data path
    // vsg/share/vsgExamples/textures/
    chrono::SetChronoDataPath("/home/thomas/Code/seabed_sim/chrono/data/");

    // -----------------------------
    // 1) Physics system
    // -----------------------------
    ChSystemSMC sys;
    // ChSystemMulticoreSMC sys;
    // sys.SetNumThreads(std::thread::hardware_concurrency());
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // pick Bullet collision
    // sys.SetCollisionSystemType(chrono::ChCollisionSystem::Type::MULTICORE);
    sys.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.6f);
    mat->SetRestitution(0.1f);

    // -----------------------------
    // 2) FEA terrain (fixed ground)
    // -----------------------------
    FEATerrain ground(&sys);
    ground.SetSoilParametersFEA(
        /*rho*/            1600.0,     // kg/m^3
        /*Emod*/           2.0e6,      // Pa
        /*nu*/             0.3,        // -
        /*yield_stress*/   2.0e4,      // Pa
        /*hardening_slope*/1.0e5,      // Pa
        /*friction_angle*/ 30.0 * CH_DEG_TO_RAD,
        /*dilatancy_angle*/ 0.0 * CH_DEG_TO_RAD
    );

    ChVector3d start(-terrain_X/2, -terrain_Y/2, -H);         // lower-left-bottom corner
    ChVector3d size ( terrain_X,    terrain_Y,    H);
    ChVector3i nelems(20,    10,     4);        // elements in x,y,z
    
    ground.Initialize(start, size, nelems);

    // FEA doesn't create a visualized thing so we need to create it
    auto mesh = ground.GetMesh();  // FEATerrain exposes its internal ChMesh
    // sys.Add(mesh); // redunant since it's already added

    auto vis_mesh = chrono_types::make_shared<chrono::ChVisualShapeFEA>(mesh);
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    vis_mesh->SetWireframe(true);          // try true if you want to see the grid
    // vis_mesh->SetColorscaleMinMax(0.0, 5.50);
    // vis_mesh->SetSmoothFaces(true);
    // vis_mesh->SetShrinkElements(true, 0.85);
    // vis_mesh->SetSmoothFaces(true);
    vis_mesh->SetDrawInUndeformedReference(true);

    mesh->AddVisualShapeFEA(vis_mesh);

    
    // Optional: set some contact material properties (simple)
    // (Chrono will use defaults if you skip this)

    // -----------------------------
    // 3) A falling object to see motion
    // -----------------------------
    for (int i = 0; i < 2; i++) { // 300; i++) {
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
    // vis->AddDirectionalLight(ChVector3d(-1, -1, -1), ChColor(1, 1, 1), 1.5);
    // vis->AddAmbientLight(ChColor(0.3f, 0.3f, 0.35f));
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
    // 5) Solver settings
    // -----------------------------
    sys.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    /* seems to crash
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(40);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);
    solver->SetVerbose(false);
*/
    /* balls don't render with this solver
    auto solver = chrono_types::make_shared<ChSolverGMRES>();
    solver->SetMaxIterations(200);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    sys.SetSolver(solver);
    */

    // -----------------------------
    // 6) Sim loop
    // -----------------------------
    const double step = 1e-2;
    const int spf = 2;
    ChRealtimeStepTimer realtime;

    while (vis->Run()) {
        for (int i = 0; i < spf; i++) {
            double t = sys.GetChTime();
            // advance the terrain
            ground.Synchronize(t);
            ground.Advance(step);

            // Advance dynamics
            sys.DoStepDynamics(step);
        }

        // Render
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Optional real-time pacing
        realtime.Spin(step);
    }

    return 0;
}

