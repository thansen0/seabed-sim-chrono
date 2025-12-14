#include <memory>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChRealtimeStep.h"

// Visualization (VSG)
#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;

int main() {
    // -----------------------------
    // 1) Physics system
    // -----------------------------
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.6f);
    mat->SetRestitution(0.1f);

    // -----------------------------
    // 2) Rigid terrain (fixed ground)
    // -----------------------------
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(
        40.0, 40.0, 1.0,   // size (x,y,z)
        1000.0,            // density (irrelevant since fixed)
        true,              // visual shape
        true,              // collision shape
        mat
    );
    ground->SetFixed(true);
    ground->SetPos(ChVector3d(0, 0, -0.5));  // top surface at z=0
    sys.Add(ground);

    // Optional: set some contact material properties (simple)
    // (Chrono will use defaults if you skip this)

    // -----------------------------
    // 3) A falling object to see motion
    // -----------------------------
    auto ball = chrono_types::make_shared<ChBodyEasySphere>(
        0.35,              // radius
        1000.0,            // density
        true,              // visual
        true,               // collision
        mat
    );
    ball->SetPos(ChVector3d(0, 0, 2.0));
    sys.Add(ball);

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

