
#include "raylib.h"

#include "rlImGui.h"
#include "imgui.h"

#include "core/Config.h"
#include "monde/Monde.h"
#include "robot/Robot.h"
#include "cible/Cible.h"
#include "input/CameraCtrl.h"
#include "input/ManualControlleur.h"
#include "rendering/WorldRender.h"
#include "interface/Dashboard.h"
#include "physics/CollisionChecker.h"
#include "navigation/NavigationController.h"
#include "monde/EvilAhrEvent.h"

#include <algorithm>
#include <cmath>
#include <vector>

int main()
{
    // ============================================
    // INITIALISATION DE LA FENETRE
    // ============================================

    const int screenWidth = 1280;
    const int screenHeight = 800;

    InitWindow(
        screenWidth,
        screenHeight,
        "Robot Navigation - 3D"
    );

    SetTargetFPS(60);

    rlImGuiSetup(true);

    // ============================================
    // CONFIGURATION DE LA SIMULATION
    // ============================================

    SimulationConfig config;

    const Vector3 initialRobotPosition = {
        -17.0f,
        0.65f,
        -16.0f
    };

    const Vector3 initialTargetPosition = {
        17.0f,
        0.6f,
        16.0f
    };

    // ============================================
    // CREATION DES OBJETS
    // ============================================

    World world(config.terrain);

    Robot robot(
        config.robot,
        initialRobotPosition
    );

    Target target(initialTargetPosition);

    ManualController manualController;

    CameraController cameraController;

    NavigationController navigationController;
    EvilAhrEvent evilAhrEvent;

    // Génération des obstacles dans le monde.
    world.GenerateObstacles(
        config.obstacleGeneration,
        config.robot,
        initialRobotPosition,
        initialTargetPosition
    );

    // ============================================
    // INITIALISATION DU PLANIFICATEUR GLOBAL
    // ============================================

    if (!navigationController.Initialize(
        world,
        robot,
        target
    ))
    {
        TraceLog(
            LOG_ERROR,
            "Global navigation initialization failed."
        );
    }
    else
    {
        TraceLog(
            LOG_INFO,
            "Global navigation initialized successfully."
        );
    }

    // ============================================
    // CAMERA 3D
    // ============================================

    Camera3D camera = { 0 };

    camera.position = {
        27.0f,
        34.0f,
        37.0f
    };

    camera.target = {
        0.0f,
        0.0f,
        0.0f
    };

    camera.up = {
        0.0f,
        1.0f,
        0.0f
    };

    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetMouseCursor(MOUSE_CURSOR_DEFAULT);

    // ============================================
    // BOUCLE PRINCIPALE
    // ============================================

    while (!WindowShouldClose())
    {
        const float dt = GetFrameTime();

        // ----------------------------------------
        // CONTROLES
        // ----------------------------------------

        const bool uiCapturesKeyboard =
            ImGui::GetIO().WantCaptureKeyboard;

        const bool uiCapturesMouse =
            ImGui::GetIO().WantCaptureMouse;

        manualController.Update(
            robot,
            target,
            dt,
            uiCapturesKeyboard
        );

        cameraController.Update(
            camera,
            dt,
            uiCapturesKeyboard,
            uiCapturesMouse
        );

        // ----------------------------------------
        // MISE A JOUR DE LA SIMULATION
        // ----------------------------------------

        //robot.Update(dt);

        // ============================================
        // NAVIGATION AUTOMATIQUE
        // ============================================

        if (manualController.IsAutomaticNavigation())
        {
            const Vector2 desiredVelocity =
                navigationController.CalculateDesiredVelocity(
                    robot,
                    target,
                    dt,
                    config.navigation,
                    world
                );

            robot.SetVelocity(desiredVelocity);
        }
        
        // ============================================
        // VERIFICATION DE SECURITE DU DEPLACEMENT
        // AVEC GLISSEMENT SUR LES AXES X ET Z
        // ============================================

        const Vector3 currentPosition = robot.GetPosition();
        const Vector2 requestedVelocity = robot.GetVelocity();

        constexpr float maxSubstep = 0.02f;

        const std::vector<Obstacle>& currentObstacles =
            world.GetObstacles();

        // Vérifie un déplacement selon une vitesse donnée.
        auto IsPathSafe = [&](Vector2 candidateVelocity) -> bool
        {
            if (dt <= 0.0f)
                return true;

            const int steps = static_cast<int>(
                std::ceil(dt / maxSubstep)
            );

            const float substepDuration = dt / steps;

            std::vector<Obstacle> predictedObstacles;
            predictedObstacles.reserve(currentObstacles.size());

            for (int i = 0; i < steps; ++i)
            {
                const float startTime = i * substepDuration;
                const float endTime = (i + 1) * substepDuration;
                const float middleTime = (startTime + endTime) * 0.5f;

                Vector3 segmentStart = currentPosition;
                segmentStart.x += candidateVelocity.x * startTime;
                segmentStart.z += candidateVelocity.y * startTime;

                Vector3 segmentEnd = currentPosition;
                segmentEnd.x += candidateVelocity.x * endTime;
                segmentEnd.z += candidateVelocity.y * endTime;

                predictedObstacles.clear();

                for (const Obstacle& obstacle : currentObstacles)
                {
                    predictedObstacles.push_back(
                        world.GetPredictedObstacleState(
                            obstacle,
                            middleTime
                        )
                    );
                }

                if (!CollisionChecker::IsMovementSafe(
                    segmentStart,
                    segmentEnd,
                    robot.GetRadius(),
                    world.GetTerrain(),
                    predictedObstacles
                ))
                {
                    return false;
                }
            }

            return true;
        };

        Vector2 acceptedVelocity = requestedVelocity;

        if (!IsPathSafe(requestedVelocity))
        {
            const Vector2 velocityX = {
                requestedVelocity.x,
                0.0f
            };
            const Vector2 velocityZ = {
                0.0f,
                requestedVelocity.y
            };

            const bool xIsSafe = IsPathSafe(velocityX);
            const bool zIsSafe = IsPathSafe(velocityZ);

            if (xIsSafe && zIsSafe)
            {
                acceptedVelocity =
                    std::abs(requestedVelocity.x) >=
                    std::abs(requestedVelocity.y)
                    ? velocityX
                    : velocityZ;
            }
            else if (xIsSafe)
            {
                acceptedVelocity = velocityX;
            }
            else if (zIsSafe)
            {
                acceptedVelocity = velocityZ;
            }
            else
            {
                acceptedVelocity = { 0.0f, 0.0f };
            }
        }

        robot.SetVelocity(acceptedVelocity);

        robot.Update(dt);
        target.Update(dt);
        world.Update(dt);

        evilAhrEvent.Update(
            world,
            robot,
            target,
            navigationController,
            config.obstacleGeneration,
            dt
        );

        navigationController.Update(
            world,
            robot,
            target,
            config.navigation,
            dt
        );

        const char* navigationStatus = "IDLE";

        switch (navigationController.GetState())
        {
        case NavigationState::Idle:
            navigationStatus = "IDLE";
            break;

        case NavigationState::Moving:
            navigationStatus = "MOVING";
            break;

        case NavigationState::TargetReached:
            navigationStatus = "TARGET REACHED";
            break;
        }

        // ----------------------------------------
        // AFFICHAGE
        // ----------------------------------------

        BeginDrawing();

        ClearBackground(
            Color{ 28, 31, 38, 255 }
        );

        DrawWorld3D(
            world,
            robot,
            target,
            camera,
            navigationController
        );

        // Informations minimales à l'écran
        DrawFPS(15, 15);

        DrawText(
            manualController.IsControllingTarget()
                ? "CONTROLE : CIBLE"
                : "CONTROLE : ROBOT",
            15,
            45,
            20,
            RAYWHITE
        );

        DrawText(
            "T : changer de controle | Fleches : deplacer",
            15,
            75,
            16,
            LIGHTGRAY
        );

        DrawText(
            TextFormat("NAVIGATION : %s", navigationStatus),
            15, 105, 18, YELLOW
        );

        DrawText(
            TextFormat(
                "DISTANCE : %.2f m",
                navigationController.GetDistanceToTarget(robot, target)
            ),
            15, 130, 18, RAYWHITE
        );

        rlImGuiBegin();

        DrawDashboard(
            world,
            robot,
            target,
            manualController,
            navigationController,
            evilAhrEvent,
            config
        );

        rlImGuiEnd();

        EndDrawing();
    }

    // ============================================
    // FERMETURE
    // ============================================

    rlImGuiShutdown();

    CloseWindow();

    return 0;
}
