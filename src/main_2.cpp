// Variante avec suivi spatio-temporel actif. Le fichier original reste intact.
// On reutilise les types et fonctions de recherche du programme existant,
// tout en remplaçant son point d'entrée par un contrôleur unique.
#define main RobotNavigationLegacyMain
#include "main.cpp"
#undef main

Vector3 PredictTimedRobotPosition(
    Vector3 position,
    const std::vector<TimedWaypoint>& path,
    int pathIndex,
    float elapsed,
    float robotSpeed,
    float futureTime)
{
    int index = pathIndex;
    float timeline = elapsed;
    float remaining = futureTime;
    constexpr float sampleStep = 0.025f;

    while (remaining > 0.0001f)
    {
        while (index < static_cast<int>(path.size()) &&
               timeline + 0.0001f >= path[index].arrivalTime)
        {
            position = path[index].position;
            ++index;
        }

        const float step = std::min(sampleStep, remaining);

        if (index < static_cast<int>(path.size()) && !path[index].wait)
        {
            const Vector3 waypoint = path[index].position;
            const float dx = waypoint.x - position.x;
            const float dz = waypoint.z - position.z;
            const float distance = std::sqrt(dx * dx + dz * dz);

            if (distance > 0.001f)
            {
                const float travel = std::min(robotSpeed * step, distance);
                position.x += dx / distance * travel;
                position.z += dz / distance * travel;
            }
        }

        timeline += step;
        remaining -= step;
    }

    while (index < static_cast<int>(path.size()) &&
           timeline + 0.0001f >= path[index].arrivalTime)
    {
        position = path[index].position;
        ++index;
    }

    return position;
}

bool IsTimedPathSafeInFuture(
    Vector3 robot,
    const std::vector<TimedWaypoint>& path,
    int pathIndex,
    float elapsed,
    const std::vector<Obstacle>& obstacles,
    float robotSpeed,
    float horizon,
    float timeStep,
    float* conflictTime,
    int* conflictObstacleIndex)
{
    if (path.empty() || robotSpeed <= 0.0f || timeStep <= 0.0f || horizon < 0.0f)
        return false;

    for (float t = 0.0f; t <= horizon + 0.0001f; t += timeStep)
    {
        const Vector3 futureRobot = PredictTimedRobotPosition(
            robot, path, pathIndex, elapsed, robotSpeed, t);

        for (std::size_t i = 0; i < obstacles.size(); ++i)
        {
            const Vector3 futureObstacle = PredictObstaclePosition(obstacles[i], t);
            const float halfWidth = obstacles[i].size.x * 0.5f + ROBOT_RADIUS;
            const float halfDepth = obstacles[i].size.z * 0.5f + ROBOT_RADIUS;

            if (std::abs(futureRobot.x - futureObstacle.x) <= halfWidth &&
                std::abs(futureRobot.z - futureObstacle.z) <= halfDepth)
            {
                if (conflictTime != nullptr)
                    *conflictTime = t;
                if (conflictObstacleIndex != nullptr)
                    *conflictObstacleIndex = static_cast<int>(i);
                return false;
            }
        }
    }

    return true;
}

int main()
{
    const int screenWidth = 1280;
    const int screenHeight = 800;
    InitWindow(screenWidth, screenHeight, "Robot Navigation - Space-Time A*");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 27.0f, 34.0f, 37.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    constexpr float floorSize = 40.0f;
    constexpr float robotSpeed = 3.0f;
    constexpr float manualSpeed = 4.0f;
    constexpr float replanInterval = 0.75f;
    constexpr float predictionHorizon = 2.0f;

    std::vector<Obstacle> obstacles = {
        {{-12.0f, 1.0f, -10.0f}, {3.0f, 2.0f, 4.0f}, {4.0f, 3.0f}},
        {{ -6.0f, 0.75f, -13.0f}, {4.0f, 1.5f, 2.0f}, {0.0f, 0.0f}},
        {{  1.0f, 1.5f, -12.0f}, {2.5f, 3.0f, 5.0f}, {0.0f, 0.0f}},
        {{  9.0f, 1.0f, -13.0f}, {5.0f, 2.0f, 2.0f}, {0.0f, 0.0f}},
        {{-14.0f, 1.5f, -2.0f}, {2.0f, 3.0f, 6.0f}, {0.0f, 0.0f}},
        {{ -7.0f, 0.75f,  0.0f}, {5.0f, 1.5f, 2.0f}, {0.0f, 0.0f}},
        {{  2.0f, 1.0f,  -3.0f}, {3.0f, 2.0f, 4.0f}, {0.0f, 0.0f}},
        {{ 11.0f, 1.5f,   0.0f}, {2.0f, 3.0f, 7.0f}, {0.0f, 0.0f}},
        {{-12.0f, 1.0f,   9.0f}, {5.0f, 2.0f, 2.0f}, {0.0f, 0.0f}},
        {{ -4.0f, 1.5f,   8.0f}, {2.0f, 3.0f, 5.0f}, {0.0f, 0.0f}},
        {{  4.0f, 0.75f,  12.0f}, {4.0f, 1.5f, 2.0f}, {0.0f, 0.0f}},
        {{ 12.0f, 1.0f,   10.0f}, {3.0f, 2.0f, 4.0f}, {0.0f, 0.0f}}
    };

    Vector3 robot = {-17.0f, 0.45f, -16.0f};
    Vector3 target = {17.0f, 0.15f, 16.0f};
    Vector2 targetVelocity = {0.0f, 0.0f};
    bool controlTarget = false;
    bool autoMode = false;

    std::vector<TimedWaypoint> timedPath;
    int timedPathIndex = 1;
    float timedPathElapsed = 0.0f;
    float replanTimer = 0.0f;
    float futureConflictTime = 0.0f;
    int futureObstacleIndex = -1;
    bool futurePathSafe = true;

    auto planTimedPath = [&](Vector3 destination)
    {
        timedPath = FindPathSpaceTimeAStar(
            robot, destination, obstacles, robotSpeed, 0.1f, 12.0f);

        if (timedPath.empty())
            timedPath = FindPathSpaceTimeAStar(
                robot, target, obstacles, robotSpeed, 0.1f, 12.0f);

        timedPathIndex = 1;
        timedPathElapsed = 0.0f;
        replanTimer = 0.0f;
    };

    const Rectangle autoButton = {20, 110, 190, 42};

    while (!WindowShouldClose())
    {
        const float dt = GetFrameTime();

        // Mise à jour de l'obstacle mobile avec rebond sur les limites.
        Obstacle& movingObstacle = obstacles[0];
        movingObstacle.position.x += movingObstacle.velocity.x * dt;
        movingObstacle.position.z += movingObstacle.velocity.y * dt;

        const float minX = MAP_MIN + movingObstacle.size.x * 0.5f;
        const float maxX = -MAP_MIN - movingObstacle.size.x * 0.5f;
        const float minZ = MAP_MIN + movingObstacle.size.z * 0.5f;
        const float maxZ = -MAP_MIN - movingObstacle.size.z * 0.5f;

        if (movingObstacle.position.x < minX)
        {
            movingObstacle.position.x = minX;
            movingObstacle.velocity.x = std::abs(movingObstacle.velocity.x);
        }
        else if (movingObstacle.position.x > maxX)
        {
            movingObstacle.position.x = maxX;
            movingObstacle.velocity.x = -std::abs(movingObstacle.velocity.x);
        }

        if (movingObstacle.position.z < minZ)
        {
            movingObstacle.position.z = minZ;
            movingObstacle.velocity.y = std::abs(movingObstacle.velocity.y);
        }
        else if (movingObstacle.position.z > maxZ)
        {
            movingObstacle.position.z = maxZ;
            movingObstacle.velocity.y = -std::abs(movingObstacle.velocity.y);
        }

        if (IsKeyPressed(KEY_M))
            controlTarget = !controlTarget;

        const bool canMoveManually = !autoMode || controlTarget;
        if (canMoveManually)
        {
            Vector3& controlledObject = controlTarget ? target : robot;
            float dx = 0.0f;
            float dz = 0.0f;
            if (IsKeyDown(KEY_UP)) dz -= 1.0f;
            if (IsKeyDown(KEY_DOWN)) dz += 1.0f;
            if (IsKeyDown(KEY_LEFT)) dx -= 1.0f;
            if (IsKeyDown(KEY_RIGHT)) dx += 1.0f;

            const float length = std::sqrt(dx * dx + dz * dz);
            if (length > 0.0f)
            {
                dx /= length;
                dz /= length;
                controlledObject.x = Clamp(
                    controlledObject.x + dx * manualSpeed * dt, -19.0f, 19.0f);
                controlledObject.z = Clamp(
                    controlledObject.z + dz * manualSpeed * dt, -19.0f, 19.0f);

                if (controlTarget)
                {
                    targetVelocity = {dx * manualSpeed, dz * manualSpeed};
                    if (autoMode)
                        replanTimer = replanInterval;
                }
            }
            else if (controlTarget)
            {
                targetVelocity = {0.0f, 0.0f};
            }
        }

        if (!controlTarget)
            targetVelocity = {0.0f, 0.0f};

        // Contrôles caméra.
        if (IsKeyDown(KEY_W)) camera.position.z -= 0.25f;
        if (IsKeyDown(KEY_S)) camera.position.z += 0.25f;
        if (IsKeyDown(KEY_A)) camera.position.x -= 0.25f;
        if (IsKeyDown(KEY_D)) camera.position.x += 0.25f;
        if (IsKeyDown(KEY_E)) camera.position.y += 0.25f;
        if (IsKeyDown(KEY_Q)) camera.position.y -= 0.25f;

        const float wheel = GetMouseWheelMove();
        if (wheel != 0.0f)
        {
            Vector3 direction = Vector3Subtract(camera.target, camera.position);
            direction = Vector3Scale(Vector3Normalize(direction), wheel * 1.5f);
            camera.position = Vector3Add(camera.position, direction);
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            const Vector2 mouseDelta = GetMouseDelta();
            Vector3 offset = Vector3Subtract(camera.position, camera.target);
            offset = Vector3RotateByAxisAngle(
                offset, {0.0f, 1.0f, 0.0f}, -mouseDelta.x * 0.005f);
            camera.position = Vector3Add(camera.target, offset);
        }

        const Vector2 mouse = GetMousePosition();
        const bool clickAuto = IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
                               CheckCollisionPointRec(mouse, autoButton);
        const bool toggleAuto = clickAuto || IsKeyPressed(KEY_SPACE);
        if (toggleAuto)
        {
            autoMode = !autoMode;
            if (autoMode)
            {
                Vector3 predictedTarget = PredictPosition(
                    target, targetVelocity, predictionHorizon);
                predictedTarget.x = Clamp(predictedTarget.x, -19.0f, 19.0f);
                predictedTarget.z = Clamp(predictedTarget.z, -19.0f, 19.0f);
                planTimedPath(predictedTarget);
            }
            else
            {
                timedPath.clear();
                timedPathIndex = 1;
                timedPathElapsed = 0.0f;
                replanTimer = 0.0f;
            }
        }

        Vector3 predictedTarget = PredictPosition(
            target, targetVelocity, predictionHorizon);
        predictedTarget.x = Clamp(predictedTarget.x, -19.0f, 19.0f);
        predictedTarget.z = Clamp(predictedTarget.z, -19.0f, 19.0f);

        if (autoMode && !timedPath.empty())
        {
            futurePathSafe = IsTimedPathSafeInFuture(
                robot, timedPath, timedPathIndex, timedPathElapsed,
                obstacles, robotSpeed, 2.0f, 0.05f,
                &futureConflictTime, &futureObstacleIndex);
            if (!futurePathSafe)
                replanTimer = replanInterval;
        }
        else
        {
            futurePathSafe = true;
            futureConflictTime = 0.0f;
            futureObstacleIndex = -1;
        }

        if (autoMode)
        {
            replanTimer += dt;
            if (timedPath.empty() || replanTimer >= replanInterval)
            {
                predictedTarget = PredictPosition(
                    target, targetVelocity, predictionHorizon);
                predictedTarget.x = Clamp(predictedTarget.x, -19.0f, 19.0f);
                predictedTarget.z = Clamp(predictedTarget.z, -19.0f, 19.0f);
                planTimedPath(predictedTarget);
            }

            if (timedPath.empty())
            {
                futurePathSafe = false;
            }
            else if (timedPathIndex >= static_cast<int>(timedPath.size()))
            {
                autoMode = false;
                timedPath.clear();
            }
            else
            {
                timedPathElapsed += dt;
                while (timedPathIndex < static_cast<int>(timedPath.size()) &&
                       timedPathElapsed >= timedPath[timedPathIndex].arrivalTime)
                {
                    robot.x = timedPath[timedPathIndex].position.x;
                    robot.z = timedPath[timedPathIndex].position.z;
                    ++timedPathIndex;
                }

                if (timedPathIndex >= static_cast<int>(timedPath.size()))
                {
                    autoMode = false;
                    timedPath.clear();
                }
                else if (!timedPath[timedPathIndex].wait)
                {
                    const Vector3 waypoint = timedPath[timedPathIndex].position;
                    const float dx = waypoint.x - robot.x;
                    const float dz = waypoint.z - robot.z;
                    const float distance = std::sqrt(dx * dx + dz * dz);
                    if (distance > 0.001f)
                    {
                        const float step = std::min(robotSpeed * dt, distance);
                        robot.x += dx / distance * step;
                        robot.z += dz / distance * step;
                    }
                }
            }
        }

        BeginDrawing();
        ClearBackground({235, 240, 245, 255});
        BeginMode3D(camera);

        DrawPlane({0.0f, -0.05f, 0.0f}, {floorSize, floorSize},
                  {220, 226, 218, 255});
        DrawGrid(40, 1.0f);
        DrawCubeWires({0.0f, 0.5f, 0.0f}, floorSize, 1.0f, floorSize, DARKGRAY);

        for (const Obstacle& obstacle : obstacles)
            DrawCube(obstacle.position, obstacle.size.x, obstacle.size.y,
                     obstacle.size.z, GRAY);

        if (timedPath.size() >= 2)
        {
            Vector3 previous = {robot.x, 0.10f, robot.z};
            for (int i = timedPathIndex; i < static_cast<int>(timedPath.size()); ++i)
            {
                DrawLine3D(previous, timedPath[i].position,
                           timedPath[i].wait ? ORANGE : SKYBLUE);
                previous = timedPath[i].position;
            }
        }

        DrawCube(robot, 0.9f, 0.55f, 0.9f, BLUE);
        DrawCubeWires(robot, 0.9f, 0.55f, 0.9f, DARKBLUE);
        DrawCylinder(target, 0.35f, 0.35f, 0.3f, 20, GREEN);
        DrawCylinderWires(target, 0.35f, 0.35f, 0.3f, 20, DARKGREEN);
        DrawSphere(predictedTarget, 0.35f, YELLOW);
        EndMode3D();

        const bool hovered = CheckCollisionPointRec(mouse, autoButton);
        const Color buttonColor = autoMode
            ? (hovered ? DARKGREEN : GREEN)
            : (hovered ? DARKBLUE : BLUE);
        DrawRectangleRec(autoButton, buttonColor);
        DrawRectangleLinesEx(autoButton, 2.0f, DARKGRAY);
        DrawText("AUTO ON/OFF (SPACE)", 30, 121, 18, WHITE);
        DrawText("ROBOT NAVIGATION - SPACE-TIME A*", 20, 20, 24, DARKBLUE);
        DrawText(controlTarget ? "Mode : CIBLE (M)" : "Mode : ROBOT (M)",
                 20, 65, 18, MAROON);
        DrawText(autoMode ? "Suivi automatique actif" : "Mode manuel",
                 20, 85, 18, DARKGRAY);

        if (futurePathSafe)
            DrawText("TRAJET TEMPOREL : SUR", 230, 120, 20, GREEN);
        else if (futureObstacleIndex >= 0)
            DrawText(TextFormat("DANGER : obstacle %d dans %.2f s",
                     futureObstacleIndex, futureConflictTime),
                     230, 120, 20, RED);
        else
            DrawText("Aucun chemin spatio-temporel disponible", 230, 120, 20, RED);

        DrawText("Fleches : deplacer | M : robot/cible | Espace ou bouton : AUTO",
                 20, screenHeight - 35, 17, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
