#include "core/Config.h"
#include "navigation/LocalGrid.h"
#include "navigation/LocalPlanner.h"
#include "navigation/NavigationController.h"
#include "input/ManualControlleur.h"
#include "monde/Monde.h"
#include "physics/CollisionChecker.h"
#include "robot/Robot.h"
#include "cible/Cible.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace
{
    constexpr float TEST_DT = 1.0f / 60.0f;

    void Check(bool condition, const char* message)
    {
        if (condition)
            return;

        std::cerr << "FAILED: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }

    float Speed(Vector2 velocity)
    {
        return std::sqrt(
            velocity.x * velocity.x +
            velocity.y * velocity.y
        );
    }

    TerrainConfig MakeTerrain()
    {
        TerrainConfig terrain;
        terrain.cellsX = 80;
        terrain.cellsZ = 80;
        terrain.cellSize = 1.0f;
        return terrain;
    }

    struct BehaviorMetrics
    {
        bool initialized = false;
        bool reachedTarget = false;
        bool collision = false;
        int firstCollisionFrame = -1;
        float firstCollisionRobotX = 0.0f;
        float firstCollisionObstacleX = 0.0f;
        int firstThreatFrame = -1;
        int firstEvasiveFrame = -1;
        int lateralDirectionChanges = 0;
        std::vector<int> lateralSwitchFrames;
        std::uint64_t safePrefixDecisions = 0;
        std::uint64_t rejectedMovements = 0;
        float distanceTravelled = 0.0f;
        Vector3 finalPosition = { 0.0f, 0.0f, 0.0f };
        float finalSpeed = 0.0f;
        int longestStoppedFrames = 0;
        std::size_t globalPathSize = 0;
        std::uint64_t temporalSearches = 0;
        std::uint64_t evaluatedTrajectories = 0;
        std::uint64_t checkedSegments = 0;
        double decisionMilliseconds = 0.0;
        double maximumDecisionMilliseconds = 0.0;
    };

    BehaviorMetrics RunBehaviorScenario(
        World& world,
        float startZ,
        float goalZ = 0.0f,
        int frameCount = 1000,
        std::uint32_t obstacleToRemove = 0,
        int removeAtFrame = -1,
        float robotSpeed = 4.0f
    )
    {
        RobotConfig robotConfig;
        robotConfig.speed = robotSpeed;
        Robot robot(
            robotConfig,
            { -15.0f, 0.65f, startZ }
        );
        Target target({ 15.0f, 0.6f, goalZ });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.3f;

        BehaviorMetrics metrics;
        metrics.initialized =
            navigation.Initialize(world, robot, target);
        if (!metrics.initialized)
            return metrics;

        int currentStoppedFrames = 0;
        int previousLateralDirection = 0;
        const Vector3 initialPosition = robot.GetPosition();
        for (int frame = 0; frame < frameCount; ++frame)
        {
            if (frame == removeAtFrame &&
                obstacleToRemove != 0)
            {
                world.RemoveObstacle(obstacleToRemove);
            }

            const auto decisionStart =
                std::chrono::steady_clock::now();
            const Vector2 requestedVelocity =
                navigation.CalculateDesiredVelocity(
                    robot,
                    target,
                    TEST_DT,
                    navigationConfig,
                    world
                );
            const auto decisionEnd =
                std::chrono::steady_clock::now();
            metrics.decisionMilliseconds +=
                std::chrono::duration<double, std::milli>(
                    decisionEnd - decisionStart
                ).count();
            metrics.maximumDecisionMilliseconds = std::max(
                metrics.maximumDecisionMilliseconds,
                std::chrono::duration<double, std::milli>(
                    decisionEnd - decisionStart
                ).count()
            );

            const LocalPlannerDiagnostics& plannerDiagnostics =
                navigation.GetLocalPlannerDiagnostics();
            metrics.evaluatedTrajectories +=
                plannerDiagnostics.evaluatedTrajectories;
            metrics.checkedSegments +=
                plannerDiagnostics.checkedSegments;
            if (plannerDiagnostics.selectedSafePrefix)
                ++metrics.safePrefixDecisions;
            metrics.temporalSearches =
                navigation.GetTemporalDiagnostics().searchesRun;
            if (metrics.firstThreatFrame < 0 &&
                navigation.GetTemporalDiagnostics().threatsDetected > 0)
            {
                metrics.firstThreatFrame = frame;
            }
            const bool threatActive =
                navigation.GetTemporalDiagnostics().threatsDetected > 0;
            if (metrics.firstEvasiveFrame < 0 &&
                threatActive &&
                std::abs(requestedVelocity.y) > 0.25f)
            {
                metrics.firstEvasiveFrame = frame;
            }

            const bool inInitialEvasionWindow =
                metrics.firstThreatFrame >= 0 &&
                frame - metrics.firstThreatFrame <= 60;
            const int lateralDirection =
                threatActive && inInitialEvasionWindow
                ? plannerDiagnostics.selectedAvoidanceSide
                : 0;
            if (lateralDirection != 0)
            {
                if (previousLateralDirection != 0 &&
                    lateralDirection != previousLateralDirection)
                {
                    ++metrics.lateralDirectionChanges;
                    metrics.lateralSwitchFrames.push_back(frame);
                }
                previousLateralDirection = lateralDirection;
            }
            else if (!threatActive)
            {
                previousLateralDirection = 0;
            }

            const Vector3 start = robot.GetPosition();
            const Vector3 end = {
                start.x + requestedVelocity.x * TEST_DT,
                start.y,
                start.z + requestedVelocity.y * TEST_DT
            };
            std::vector<Obstacle> predictedObstacles;
            predictedObstacles.reserve(world.GetObstacles().size());
            for (const Obstacle& obstacle : world.GetObstacles())
            {
                predictedObstacles.push_back(
                    world.GetPredictedObstacleState(
                        obstacle,
                        TEST_DT * 0.5f
                    )
                );
            }

            const bool safe = CollisionChecker::IsMovementSafe(
                start,
                end,
                robot.GetRadius(),
                world.GetTerrain(),
                predictedObstacles
            );
            if (!safe)
            {
                ++metrics.rejectedMovements;
                robot.SetVelocity({ 0.0f, 0.0f });
            }
            else
            {
                robot.SetVelocity(requestedVelocity);
            }

            if (Speed(robot.GetVelocity()) < 0.05f)
            {
                ++currentStoppedFrames;
                metrics.longestStoppedFrames = std::max(
                    metrics.longestStoppedFrames,
                    currentStoppedFrames
                );
            }
            else
            {
                currentStoppedFrames = 0;
            }

            robot.Update(TEST_DT);
            world.Update(TEST_DT);
            navigation.Update(
                world,
                robot,
                target,
                navigationConfig,
                TEST_DT
            );
            if (!CollisionChecker::IsPositionSafe(
                robot.GetPosition(),
                robot.GetRadius(),
                world.GetTerrain(),
                world.GetObstacles()))
            {
                metrics.collision = true;
                if (metrics.firstCollisionFrame < 0)
                {
                    metrics.firstCollisionFrame = frame;
                    metrics.firstCollisionRobotX =
                        robot.GetPosition().x;
                    if (!world.GetObstacles().empty())
                    {
                        metrics.firstCollisionObstacleX =
                            world.GetObstacles().front().position.x;
                    }
                }
            }

            if (navigation.GetState() ==
                NavigationState::TargetReached)
            {
                metrics.reachedTarget = true;
                break;
            }
        }

        const Vector3 finalPosition = robot.GetPosition();
        metrics.finalPosition = finalPosition;
        metrics.finalSpeed = Speed(robot.GetVelocity());
        metrics.globalPathSize = navigation.GetGlobalPath().size();
        metrics.distanceTravelled = std::sqrt(
            (finalPosition.x - initialPosition.x) *
                (finalPosition.x - initialPosition.x) +
            (finalPosition.z - initialPosition.z) *
                (finalPosition.z - initialPosition.z)
        );
        return metrics;
    }

    void ReportBehavior(
        const char* name,
        const BehaviorMetrics& metrics
    )
    {
        std::cout << "SCENARIO " << name
                  << " reached=" << metrics.reachedTarget
                  << " collision=" << metrics.collision
                  << " first_collision_frame="
                  << metrics.firstCollisionFrame
                  << " collision_x=("
                  << metrics.firstCollisionRobotX << ','
                  << metrics.firstCollisionObstacleX << ')'
                  << " threat_frame=" << metrics.firstThreatFrame
                  << " evade_frame=" << metrics.firstEvasiveFrame
                  << " lateral_switches="
                  << metrics.lateralDirectionChanges
                  << " prefix_fallbacks="
                  << metrics.safePrefixDecisions
                  << " rejected_moves=" << metrics.rejectedMovements
                  << " distance=" << metrics.distanceTravelled
                  << " position=(" << metrics.finalPosition.x << ','
                  << metrics.finalPosition.z << ')'
                  << " speed=" << metrics.finalSpeed
                  << " path_cells=" << metrics.globalPathSize
                  << " longest_stop_frames="
                  << metrics.longestStoppedFrames
                  << " temporal_searches=" << metrics.temporalSearches
                  << " trajectories="
                  << metrics.evaluatedTrajectories
                  << " segments=" << metrics.checkedSegments
                  << " decision_ms="
                  << metrics.decisionMilliseconds
                  << " max_decision_ms="
                  << metrics.maximumDecisionMilliseconds << '\n';
        if (!metrics.lateralSwitchFrames.empty())
        {
            std::cout << " lateral_switch_frames=";
            for (const int frame : metrics.lateralSwitchFrames)
                std::cout << frame << ',';
            std::cout << '\n';
        }
    }

    bool PathEndsAtTargetCell(
        const NavigationController& navigation,
        const Target& target
    )
    {
        const GridMap* grid = navigation.GetGlobalGrid();
        if (!grid || navigation.GetGlobalPath().empty())
            return false;

        const Vector3 position = target.GetPosition();
        GridCell targetCell;
        if (!grid->WorldToCell(position.x, position.z, targetCell))
            return false;

        const GridCell pathGoal = navigation.GetGlobalPath().back();
        return pathGoal.x == targetCell.x &&
            pathGoal.z == targetCell.z;
    }

    bool AdvanceRetargetScenario(
        World& world,
        Robot& robot,
        Target& target,
        NavigationController& navigation,
        const NavigationConfig& navigationConfig,
        double& maximumUpdateMilliseconds,
        float dt = TEST_DT
    )
    {
        const Vector2 command = navigation.CalculateDesiredVelocity(
            robot,
            target,
            dt,
            navigationConfig,
            world
        );
        const Vector3 start = robot.GetPosition();
        const Vector3 end = {
            start.x + command.x * dt,
            start.y,
            start.z + command.y * dt
        };
        constexpr float maxSubstep = 0.02f;
        const int steps = std::max(
            1,
            static_cast<int>(std::ceil(dt / maxSubstep))
        );
        const float substepDuration = dt / steps;
        std::vector<Obstacle> predictedObstacles;
        predictedObstacles.reserve(world.GetObstacles().size());
        bool safe = true;
        for (int step = 0; step < steps && safe; ++step)
        {
            const float startTime = step * substepDuration;
            const float endTime = (step + 1) * substepDuration;
            const float middleTime = (startTime + endTime) * 0.5f;
            Vector3 segmentStart = start;
            segmentStart.x += command.x * startTime;
            segmentStart.z += command.y * startTime;
            Vector3 segmentEnd = start;
            segmentEnd.x += command.x * endTime;
            segmentEnd.z += command.y * endTime;

            predictedObstacles.clear();
            for (const Obstacle& obstacle : world.GetObstacles())
            {
                predictedObstacles.push_back(
                    world.GetPredictedObstacleState(obstacle, middleTime)
                );
            }

            safe = CollisionChecker::IsMovementSafe(
                segmentStart,
                segmentEnd,
                robot.GetRadius(),
                world.GetTerrain(),
                predictedObstacles
            );
        }
        robot.SetVelocity(safe ? command : Vector2{ 0.0f, 0.0f });
        robot.Update(dt);
        target.Update(dt);
        world.Update(dt);

        const auto updateStart = std::chrono::steady_clock::now();
        navigation.Update(
            world,
            robot,
            target,
            navigationConfig,
            dt
        );
        const auto updateEnd = std::chrono::steady_clock::now();
        maximumUpdateMilliseconds = std::max(
            maximumUpdateMilliseconds,
            std::chrono::duration<double, std::milli>(
                updateEnd - updateStart
            ).count()
        );

        return safe && CollisionChecker::IsPositionSafe(
            robot.GetPosition(),
            robot.GetRadius(),
            world.GetTerrain(),
            world.GetObstacles()
        );
    }

    void TestManualSpeedLevels()
    {
        RobotConfig config;
        config.speed = 1.0f;
        Robot robot(config, { 0.0f, 0.65f, 0.0f });

        for (const float requestedSpeed : { 2.0f, 4.0f, 6.0f, 8.0f })
        {
            robot.SetMaxSpeed(requestedSpeed);
            const Vector2 command =
                ManualController::CalculateInputVelocity(
                    1.0f,
                    0.0f,
                    robot.GetMaxSpeed()
                );
            robot.SetVelocity(command);

            Check(
                std::abs(Speed(robot.GetVelocity()) - requestedSpeed) <
                    0.001f,
                "manual input immediately follows the selected speed"
            );

            const float startX = robot.GetPosition().x;
            for (int frame = 0; frame < 120; ++frame)
                robot.Update(TEST_DT);

            const float distance = robot.GetPosition().x - startX;
            Check(
                std::abs(distance - requestedSpeed * 2.0f) < 0.01f,
                "manual displacement follows the selected speed"
            );
        }

        const Vector2 diagonal =
            ManualController::CalculateInputVelocity(
                1.0f,
                1.0f,
                robot.GetMaxSpeed()
            );
        Check(
            std::abs(Speed(diagonal) - robot.GetMaxSpeed()) < 0.001f,
            "diagonal manual input does not exceed the speed limit"
        );

        std::cout << "PASS manual speed levels 2/4/6/8 m/s\n";
    }

    void TestAutomaticSpeedLevels()
    {
        const TerrainConfig terrain = MakeTerrain();
        float previousMeasuredSpeed = 0.0f;

        for (const float requestedSpeed : { 2.0f, 4.0f, 6.0f, 8.0f })
        {
            World world(terrain);
            RobotConfig robotConfig;
            robotConfig.speed = requestedSpeed;
            Robot robot(robotConfig, { -30.0f, 0.65f, 0.0f });
            Target target({ 30.0f, 0.6f, 0.0f });
            NavigationController navigation;
            NavigationConfig navigationConfig;
            navigationConfig.arrivalRadius = 0.2f;

            Check(
                navigation.Initialize(world, robot, target),
                "automatic speed test initializes navigation"
            );

            float maximumMeasuredSpeed = 0.0f;
            float steadySpeedTotal = 0.0f;
            int steadySpeedSamples = 0;
            for (int frame = 0; frame < 120; ++frame)
            {
                const Vector2 command =
                    navigation.CalculateDesiredVelocity(
                        robot,
                        target,
                        TEST_DT,
                        navigationConfig,
                        world
                    );
                robot.SetVelocity(command);
                robot.Update(TEST_DT);
                world.Update(TEST_DT);
                navigation.Update(
                    world, robot, target, navigationConfig, TEST_DT);
                maximumMeasuredSpeed = std::max(
                    maximumMeasuredSpeed,
                    Speed(robot.GetVelocity())
                );
                if (frame >= 90)
                {
                    steadySpeedTotal += Speed(robot.GetVelocity());
                    ++steadySpeedSamples;
                }
            }

            const float steadySpeed =
                steadySpeedTotal / steadySpeedSamples;
            std::cout << "AUTO_SPEED limit=" << requestedSpeed
                      << " measured=" << steadySpeed
                      << " m/s (peak " << maximumMeasuredSpeed << ")\n";
            Check(
                steadySpeed >= requestedSpeed * 0.95f &&
                    steadySpeed <= requestedSpeed + 0.01f,
                "automatic robot sustains the configured speed on a clear route"
            );
            Check(
                maximumMeasuredSpeed > previousMeasuredSpeed,
                "increasing the slider speed increases measured speed"
            );
            previousMeasuredSpeed = maximumMeasuredSpeed;
        }

        std::cout << "PASS automatic speed levels 2/4/6/8 m/s\n";
    }

    void TestAutomaticSliderChangesDuringNavigation()
    {
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        RobotConfig robotConfig;
        robotConfig.speed = 2.0f;
        Robot robot(robotConfig, { -30.0f, 0.65f, 0.0f });
        Target target({ 30.0f, 0.6f, 0.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.2f;

        Check(
            navigation.Initialize(world, robot, target),
            "live slider test initializes navigation"
        );

        for (const float speedLimit : { 2.0f, 4.0f, 6.0f, 8.0f })
        {
            robot.SetMaxSpeed(speedLimit);
            float steadySpeedTotal = 0.0f;
            for (int frame = 0; frame < 45; ++frame)
            {
                const Vector2 command =
                    navigation.CalculateDesiredVelocity(
                        robot,
                        target,
                        TEST_DT,
                        navigationConfig,
                        world
                    );
                robot.SetVelocity(command);
                robot.Update(TEST_DT);
                world.Update(TEST_DT);
                navigation.Update(
                    world, robot, target, navigationConfig, TEST_DT);
                if (frame >= 30)
                    steadySpeedTotal += Speed(robot.GetVelocity());
            }

            const float measuredSpeed = steadySpeedTotal / 15.0f;
            std::cout << "LIVE_SLIDER limit=" << speedLimit
                      << " measured=" << measuredSpeed << " m/s\n";
            Check(
                measuredSpeed >= speedLimit * 0.95f &&
                    measuredSpeed <= speedLimit + 0.01f,
                "automatic speed follows each live slider change"
            );
        }

        std::cout << "PASS live slider changes during automatic navigation\n";
    }

    void TestMovingTargetReplansGlobalRoute()
    {
        const TerrainConfig terrain = MakeTerrain();
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.4f;
        double maximumUpdateMilliseconds = 0.0;

        World slowWorld(terrain);
        RobotConfig robotConfig;
        robotConfig.speed = 8.0f;
        Robot slowRobot(robotConfig, { -20.0f, 0.65f, 0.0f });
        Target slowTarget({ 12.0f, 0.6f, 0.0f });
        NavigationController slowNavigation;
        Check(
            slowNavigation.Initialize(slowWorld, slowRobot, slowTarget),
            "slow-target navigation initializes"
        );
        slowTarget.SetVelocity(
            ManualController::CalculateInputVelocity(1.0f, 0.0f, 4.0f)
        );
        for (int frame = 0; frame < 200; ++frame)
        {
            Check(
                AdvanceRetargetScenario(
                    slowWorld,
                    slowRobot,
                    slowTarget,
                    slowNavigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "slowly moving target remains collision-free"
            );
            Check(
                PathEndsAtTargetCell(slowNavigation, slowTarget),
                "global route tracks the moving target cell"
            );
        }
        slowTarget.Stop();
        for (int frame = 0; frame < 500; ++frame)
        {
            if (!AdvanceRetargetScenario(
                slowWorld,
                slowRobot,
                slowTarget,
                slowNavigation,
                navigationConfig,
                maximumUpdateMilliseconds))
            {
                Check(false, "slow-target final approach remains safe");
            }
            if (slowNavigation.GetState() ==
                NavigationState::TargetReached)
            {
                break;
            }
        }
        Check(
            slowNavigation.GetState() == NavigationState::TargetReached,
            "robot follows a slowly moving target and reaches its final position"
        );
        std::cout << "PASS slow moving target max_update_ms="
                  << maximumUpdateMilliseconds << '\n';

        World suddenWorld(terrain);
        Robot suddenRobot(robotConfig, { -20.0f, 0.65f, 0.0f });
        Target suddenTarget({ 20.0f, 0.6f, 0.0f });
        NavigationController suddenNavigation;
        Check(
            suddenNavigation.Initialize(
                suddenWorld, suddenRobot, suddenTarget),
            "sudden-retarget navigation initializes"
        );
        for (int frame = 0; frame < 70; ++frame)
        {
            Check(
                AdvanceRetargetScenario(
                    suddenWorld,
                    suddenRobot,
                    suddenTarget,
                    suddenNavigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "pre-retarget motion remains safe"
            );
        }
        suddenTarget.SetPosition({ -30.0f, 0.6f, 8.0f });
        Check(
            AdvanceRetargetScenario(
                suddenWorld,
                suddenRobot,
                suddenTarget,
                suddenNavigation,
                navigationConfig,
                maximumUpdateMilliseconds
            ) &&
                PathEndsAtTargetCell(suddenNavigation, suddenTarget),
            "a sudden destination change replaces the global path"
        );
        for (int frame = 0;
             frame < 800 &&
                 suddenNavigation.GetState() !=
                    NavigationState::TargetReached;
             ++frame)
        {
            Check(
                AdvanceRetargetScenario(
                    suddenWorld,
                    suddenRobot,
                    suddenTarget,
                    suddenNavigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "sudden-retarget route remains safe"
            );
        }
        Check(
            suddenNavigation.GetState() == NavigationState::TargetReached,
            "robot reaches a sudden destination at the opposite terrain end"
        );
        std::cout << "PASS sudden retarget max_update_ms="
                  << maximumUpdateMilliseconds << '\n';

        World obstacleWorld(terrain);
        Obstacle wall;
        wall.position = { 0.0f, 0.5f, 0.0f };
        wall.size = { 2.0f, 1.0f, 6.0f };
        Check(obstacleWorld.AddObstacle(wall) != 0,
            "retarget obstacle registers");
        Robot obstacleRobot(robotConfig, { -20.0f, 0.65f, 0.0f });
        Target obstacleTarget({ -12.0f, 0.6f, 0.0f });
        NavigationController obstacleNavigation;
        Check(
            obstacleNavigation.Initialize(
                obstacleWorld, obstacleRobot, obstacleTarget),
            "obstacle-retarget navigation initializes"
        );
        for (int frame = 0; frame < 15; ++frame)
        {
            Check(
                AdvanceRetargetScenario(
                    obstacleWorld,
                    obstacleRobot,
                    obstacleTarget,
                    obstacleNavigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "initial obstacle-side route remains safe"
            );
        }
        obstacleTarget.SetPosition({ 20.0f, 0.6f, 0.0f });
        for (int frame = 0;
             frame < 1000 &&
                 obstacleNavigation.GetState() !=
                    NavigationState::TargetReached;
             ++frame)
        {
            Check(
                AdvanceRetargetScenario(
                    obstacleWorld,
                    obstacleRobot,
                    obstacleTarget,
                    obstacleNavigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "new route around the static obstacle remains safe"
            );
            Check(
                PathEndsAtTargetCell(obstacleNavigation, obstacleTarget),
                "obstacle detour still terminates at the current target"
            );
        }
        Check(
            obstacleNavigation.GetState() == NavigationState::TargetReached,
            "robot reaches a retargeted destination beyond a static obstacle"
        );
        std::cout << "PASS retarget behind static obstacle max_update_ms="
                  << maximumUpdateMilliseconds << '\n';
    }

    void TestRepeatedRetargetsReplaceStalePaths()
    {
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        RobotConfig robotConfig;
        robotConfig.speed = 6.0f;
        Robot robot(robotConfig, { -20.0f, 0.65f, 0.0f });
        Target target({ 20.0f, 0.6f, 0.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.4f;
        Check(
            navigation.Initialize(world, robot, target),
            "repeated-retarget navigation initializes"
        );

        double maximumUpdateMilliseconds = 0.0;
        const Vector3 rapidDestinations[] = {
            { 30.0f, 0.6f, 12.0f },
            { -30.0f, 0.6f, -12.0f },
            { 28.0f, 0.6f, -14.0f },
            { -28.0f, 0.6f, 14.0f }
        };
        for (int frame = 0; frame < 80; ++frame)
        {
            target.SetPosition(
                rapidDestinations[(frame / 3) % 4]
            );
            Check(
                AdvanceRetargetScenario(
                    world,
                    robot,
                    target,
                    navigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "rapid destination changes remain collision-free"
            );
            Check(
                PathEndsAtTargetCell(navigation, target),
                "D* Lite publishes the newest destination during rapid changes"
            );
        }

        target.SetPosition({ 25.0f, 0.6f, 10.0f });
        bool collisionFree = true;
        for (int frame = 0;
             frame < 1000 &&
                 navigation.GetState() != NavigationState::TargetReached;
             ++frame)
        {
            collisionFree = AdvanceRetargetScenario(
                world,
                robot,
                target,
                navigation,
                navigationConfig,
                maximumUpdateMilliseconds
            );
            Check(
                collisionFree &&
                    PathEndsAtTargetCell(navigation, target),
                "final route replaces all stale rapid-retarget waypoints"
            );
        }
        Check(
            navigation.GetState() == NavigationState::TargetReached,
            "robot reaches the final destination after rapid retargeting"
        );
        std::cout << "PASS rapid repeated retargets max_update_ms="
                  << maximumUpdateMilliseconds << '\n';
    }

    void TestRetargetRecoversAfterInvalidDestination()
    {
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        RobotConfig robotConfig;
        robotConfig.speed = 4.0f;
        Robot robot(robotConfig, { -20.0f, 0.65f, 0.0f });
        Target target({ 20.0f, 0.6f, 0.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.4f;
        Check(
            navigation.Initialize(world, robot, target),
            "invalid-destination recovery initializes"
        );

        double maximumUpdateMilliseconds = 0.0;
        target.SetPosition({ 50.0f, 0.6f, 0.0f });
        AdvanceRetargetScenario(
            world,
            robot,
            target,
            navigation,
            navigationConfig,
            maximumUpdateMilliseconds
        );
        Check(
            navigation.GetGlobalPath().empty(),
            "navigation does not retain a path to an out-of-grid destination"
        );

        target.SetPosition({ 20.0f, 0.6f, 0.0f });
        Check(
            AdvanceRetargetScenario(
                world,
                robot,
                target,
                navigation,
                navigationConfig,
                maximumUpdateMilliseconds
            ) &&
                PathEndsAtTargetCell(navigation, target),
            "returning to a previously used goal cell triggers a fresh path"
        );

        for (int frame = 0;
             frame < 900 &&
                 navigation.GetState() != NavigationState::TargetReached;
             ++frame)
        {
            Check(
                AdvanceRetargetScenario(
                    world,
                    robot,
                    target,
                    navigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "recovered destination remains collision-free"
            );
        }
        Check(
            navigation.GetState() == NavigationState::TargetReached,
            "navigation resumes and reaches the restored destination"
        );
        std::cout << "PASS invalid destination recovery max_update_ms="
                  << maximumUpdateMilliseconds << '\n';
    }

    void TestRetargetDuringDynamicEvasion()
    {
        constexpr float dynamicTestDt = 1.0f / 40.0f;
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        Obstacle approaching;
        approaching.position = { -4.0f, 0.5f, 0.0f };
        approaching.size = { 1.5f, 1.0f, 1.5f };
        approaching.velocity = { -3.0f, 0.0f };
        approaching.isMoving = true;
        Check(world.AddObstacle(approaching) != 0,
            "dynamic retarget threat registers");

        RobotConfig robotConfig;
        robotConfig.speed = 6.0f;
        Robot robot(robotConfig, { -15.0f, 0.65f, 0.0f });
        Target target({ 15.0f, 0.6f, 0.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.4f;
        Check(
            navigation.Initialize(world, robot, target),
            "dynamic-retarget navigation initializes"
        );

        double maximumUpdateMilliseconds = 0.0;
        bool collisionFree = true;
        for (int frame = 0;
             frame < 1200 &&
                 navigation.GetState() != NavigationState::TargetReached;
             ++frame)
        {
            if (frame == 10)
                target.SetPosition({ 15.0f, 0.6f, 8.0f });

            collisionFree = AdvanceRetargetScenario(
                world,
                robot,
                target,
                navigation,
                navigationConfig,
                maximumUpdateMilliseconds,
                dynamicTestDt
            );
            Check(
                collisionFree,
                "retargeting during dynamic evasion remains collision-free"
            );
        }
        Check(
            navigation.GetTemporalDiagnostics().searchesRun > 0,
            "dynamic threat still triggers Temporal A* during retargeting"
        );
        Check(
            PathEndsAtTargetCell(navigation, target),
            "dynamic evasion resumes toward the latest global destination"
        );
        Check(
            navigation.GetState() == NavigationState::TargetReached,
            "robot reaches the changed destination after dynamic evasion"
        );
        std::cout << "PASS retarget during dynamic evasion temporal_searches="
                  << navigation.GetTemporalDiagnostics().searchesRun
                  << " max_update_ms=" << maximumUpdateMilliseconds << '\n';
    }

    void TestRetargetWhileTemporalPathActive()
    {
        constexpr float dynamicTestDt = 1.0f / 40.0f;
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        Obstacle approaching;
        approaching.position = { -4.0f, 0.5f, 0.0f };
        approaching.size = { 1.5f, 1.0f, 1.5f };
        approaching.velocity = { -3.0f, 0.0f };
        approaching.isMoving = true;
        Check(world.AddObstacle(approaching) != 0,
            "temporal retarget threat registers");

        RobotConfig robotConfig;
        robotConfig.speed = 6.0f;
        Robot robot(robotConfig, { -15.0f, 0.65f, 0.0f });
        Target target({ 15.0f, 0.6f, 0.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.4f;
        Check(
            navigation.Initialize(world, robot, target),
            "active-temporal-path navigation initializes"
        );

        double maximumUpdateMilliseconds = 0.0;
        bool collisionFree = true;
        int framesToTemporalPath = 0;
        while (framesToTemporalPath < 180 &&
               !navigation.IsTemporalPathActive())
        {
            collisionFree = AdvanceRetargetScenario(
                world,
                robot,
                target,
                navigation,
                navigationConfig,
                maximumUpdateMilliseconds,
                dynamicTestDt
            );
            Check(
                collisionFree,
                "approach to temporal path remains collision-free"
            );
            ++framesToTemporalPath;
        }
        Check(
            navigation.IsTemporalPathActive(),
            "Temporal A* path is active before the destination moves"
        );

        const std::uint64_t searchesBeforeRetarget =
            navigation.GetTemporalDiagnostics().searchesRun;
        target.SetPosition({ 15.0f, 0.6f, 10.0f });
        collisionFree = AdvanceRetargetScenario(
            world,
            robot,
            target,
            navigation,
            navigationConfig,
            maximumUpdateMilliseconds,
            dynamicTestDt
        );
        Check(
            collisionFree &&
                PathEndsAtTargetCell(navigation, target),
            "active temporal diversion follows the newest destination"
        );
        Check(
            !navigation.IsTemporalPathActive() ||
                navigation.GetTemporalDiagnostics().searchesRun >
                    searchesBeforeRetarget,
            "old temporal path is invalidated or replaced after retargeting"
        );

        for (int frame = 0;
             frame < 1200 &&
                 navigation.GetState() != NavigationState::TargetReached;
             ++frame)
        {
            collisionFree = AdvanceRetargetScenario(
                world,
                robot,
                target,
                navigation,
                navigationConfig,
                maximumUpdateMilliseconds,
                dynamicTestDt
            );
            Check(
                collisionFree,
                "continued temporal navigation remains collision-free"
            );
        }
        Check(
            navigation.GetState() == NavigationState::TargetReached,
            "robot reaches the destination after replacing an active temporal path"
        );
        std::cout << "PASS retarget during active Temporal A* max_update_ms="
                  << maximumUpdateMilliseconds << '\n';
    }

    void TestNoGlobalPathWaitsAndRecovers()
    {
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        Obstacle wall;
        wall.position = { 0.0f, 0.5f, 0.0f };
        wall.size = { 4.0f, 1.0f, 8.0f };
        Check(world.AddObstacle(wall) != 0,
            "target-wall obstacle registers");

        RobotConfig robotConfig;
        robotConfig.speed = 4.0f;
        Robot robot(robotConfig, { -15.0f, 0.65f, 0.0f });
        Target target({ 0.0f, 0.6f, 0.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.4f;
        Check(
            !navigation.Initialize(world, robot, target),
            "target inside a static wall has no global path"
        );

        double maximumUpdateMilliseconds = 0.0;
        const Vector3 waitingPosition = robot.GetPosition();
        for (int frame = 0; frame < 60; ++frame)
        {
            Check(
                AdvanceRetargetScenario(
                    world,
                    robot,
                    target,
                    navigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "waiting at an unreachable target remains safe"
            );
        }
        Check(
            std::sqrt(
                std::pow(robot.GetPosition().x - waitingPosition.x, 2.0f) +
                std::pow(robot.GetPosition().z - waitingPosition.z, 2.0f)
            ) < 0.05f,
            "robot holds position when the target is inside a static obstacle"
        );

        target.SetPosition({ -10.0f, 0.6f, 7.0f });
        Check(
            AdvanceRetargetScenario(
                world,
                robot,
                target,
                navigation,
                navigationConfig,
                maximumUpdateMilliseconds
            ) &&
                !navigation.GetGlobalPath().empty() &&
                PathEndsAtTargetCell(navigation, target),
            "free destination immediately restores a global path"
        );
        for (int frame = 0;
             frame < 700 &&
                 navigation.GetState() != NavigationState::TargetReached;
             ++frame)
        {
            Check(
                AdvanceRetargetScenario(
                    world,
                    robot,
                    target,
                    navigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "recovered route around the wall remains safe"
            );
        }
        Check(
            navigation.GetState() == NavigationState::TargetReached,
            "normal navigation resumes after an unreachable target becomes reachable"
        );
        std::cout << "PASS blocked target wait and recovery max_update_ms="
                  << maximumUpdateMilliseconds << '\n';
    }

    void TestNoGlobalPathBehindImpassableWall()
    {
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        Obstacle wall;
        wall.position = { 0.0f, 0.5f, 0.0f };
        wall.size = { 2.0f, 1.0f, 80.0f };
        Check(world.AddObstacle(wall) != 0,
            "terrain-wide wall registers");

        RobotConfig robotConfig;
        robotConfig.speed = 4.0f;
        Robot robot(robotConfig, { -15.0f, 0.65f, 0.0f });
        Target target({ 15.0f, 0.6f, 0.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.4f;
        Check(
            !navigation.Initialize(world, robot, target),
            "target behind an impassable wall has no global path"
        );

        double maximumUpdateMilliseconds = 0.0;
        const Vector3 waitingPosition = robot.GetPosition();
        for (int frame = 0; frame < 80; ++frame)
        {
            Check(
                AdvanceRetargetScenario(
                    world,
                    robot,
                    target,
                    navigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "robot waits safely in front of an impassable wall"
            );
        }
        Check(
            std::abs(robot.GetPosition().x - waitingPosition.x) < 0.05f &&
                std::abs(robot.GetPosition().z - waitingPosition.z) < 0.05f,
            "robot does not keep following a stale path through the wall"
        );

        target.SetPosition({ -8.0f, 0.6f, 8.0f });
        Check(
            AdvanceRetargetScenario(
                world,
                robot,
                target,
                navigation,
                navigationConfig,
                maximumUpdateMilliseconds
            ) &&
                PathEndsAtTargetCell(navigation, target),
            "accessible same-side target restores D* Lite navigation"
        );
        for (int frame = 0;
             frame < 500 &&
                 navigation.GetState() != NavigationState::TargetReached;
             ++frame)
        {
            Check(
                AdvanceRetargetScenario(
                    world,
                    robot,
                    target,
                    navigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "same-side recovered route remains safe"
            );
        }
        Check(
            navigation.GetState() == NavigationState::TargetReached,
            "robot reaches a valid target after an unreachable one"
        );
        std::cout << "PASS impassable wall target transition max_update_ms="
                  << maximumUpdateMilliseconds << '\n';
    }

    void TestNoGlobalPathKeepsDynamicEvasion()
    {
        constexpr float dynamicTestDt = 1.0f / 40.0f;
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        Obstacle wall;
        wall.position = { 0.0f, 0.5f, 0.0f };
        wall.size = { 2.0f, 1.0f, 80.0f };
        Check(world.AddObstacle(wall) != 0,
            "dynamic no-path wall registers");
        Obstacle approaching;
        approaching.position = { -4.5f, 0.5f, 0.0f };
        approaching.size = { 1.5f, 1.0f, 1.5f };
        approaching.velocity = { -3.0f, 0.0f };
        approaching.isMoving = true;
        Check(world.AddObstacle(approaching) != 0,
            "approaching obstacle registers for no-path protection");

        RobotConfig robotConfig;
        robotConfig.speed = 6.0f;
        Robot robot(robotConfig, { -15.0f, 0.65f, 0.0f });
        Target target({ 15.0f, 0.6f, 0.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.4f;
        Check(
            !navigation.Initialize(world, robot, target),
            "dynamic protection scenario starts without a global path"
        );

        double maximumUpdateMilliseconds = 0.0;
        float maximumLateralDisplacement = 0.0f;
        const Vector3 initialPosition = robot.GetPosition();
        bool collisionFree = true;
        bool observedThreat = false;
        std::uint64_t temporalSearchesDuringWait = 0;
        Vector3 positionAfterThreat = initialPosition;
        for (int frame = 0; frame < 320; ++frame)
        {
            collisionFree = AdvanceRetargetScenario(
                world,
                robot,
                target,
                navigation,
                navigationConfig,
                maximumUpdateMilliseconds,
                dynamicTestDt
            );
            Check(
                collisionFree,
                "local defensive maneuver remains collision-free without a global path"
            );
            observedThreat = observedThreat ||
                navigation.GetTemporalDiagnostics().threatsDetected > 0;
            temporalSearchesDuringWait = std::max(
                temporalSearchesDuringWait,
                navigation.GetTemporalDiagnostics().searchesRun
            );
            maximumLateralDisplacement = std::max(
                maximumLateralDisplacement,
                std::abs(robot.GetPosition().z - initialPosition.z)
            );
            if (frame == 240)
                positionAfterThreat = robot.GetPosition();
        }
        Check(
            observedThreat,
            "dynamic threat remains diagnosed while global path is absent"
        );
        Check(
            temporalSearchesDuringWait == 0,
            "no global goal means Temporal A* does not search for an invalid goal"
        );
        Check(
            maximumLateralDisplacement > 0.5f,
            "LocalPlanner performs a safe local evasion while waiting"
        );
        Check(
            robot.GetPosition().x < -1.5f,
            "after evasion robot does not resume a stale route through the wall"
        );
        Check(
            std::abs(robot.GetPosition().x - positionAfterThreat.x) < 0.1f &&
                std::abs(robot.GetPosition().z - positionAfterThreat.z) < 0.1f,
            "robot settles at a stable position after the threat passes"
        );
        std::cout << "PASS no-path dynamic evasion max_lateral="
                  << maximumLateralDisplacement
                  << " max_update_ms=" << maximumUpdateMilliseconds << '\n';
    }

    void TestRepeatedReachableAndUnreachableTargets()
    {
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        Obstacle wall;
        wall.position = { 0.0f, 0.5f, 0.0f };
        wall.size = { 2.0f, 1.0f, 80.0f };
        Check(world.AddObstacle(wall) != 0,
            "repeated-transition wall registers");

        RobotConfig robotConfig;
        robotConfig.speed = 4.0f;
        Robot robot(robotConfig, { -15.0f, 0.65f, 0.0f });
        Target target({ -10.0f, 0.6f, 8.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.4f;
        Check(
            navigation.Initialize(world, robot, target),
            "repeated-transition navigation initializes"
        );
        double maximumUpdateMilliseconds = 0.0;

        for (int transition = 0; transition < 6; ++transition)
        {
            const bool accessible = transition % 2 == 0;
            target.SetPosition(
                accessible
                ? Vector3{ -10.0f, 0.6f, 8.0f }
                : Vector3{ 15.0f, 0.6f, 0.0f }
            );
            Check(
                AdvanceRetargetScenario(
                    world,
                    robot,
                    target,
                    navigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ),
                "repeated accessible/inaccessible target changes remain safe"
            );
            if (accessible)
            {
                Check(
                    !navigation.GetGlobalPath().empty() &&
                        PathEndsAtTargetCell(navigation, target),
                    "accessible transition restores the current global path"
                );
            }
            else
            {
                Check(
                    navigation.GetGlobalPath().empty(),
                    "inaccessible transition clears the previous global path"
                );
            }
        }

        target.SetPosition({ -8.0f, 0.6f, -8.0f });
        for (int frame = 0;
             frame < 700 &&
                 navigation.GetState() != NavigationState::TargetReached;
             ++frame)
        {
            Check(
                AdvanceRetargetScenario(
                    world,
                    robot,
                    target,
                    navigation,
                    navigationConfig,
                    maximumUpdateMilliseconds
                ) &&
                    PathEndsAtTargetCell(navigation, target),
                "repeated transitions leave no stale path or collision"
            );
        }
        Check(
            navigation.GetState() == NavigationState::TargetReached,
            "normal navigation resumes after repeated goal accessibility changes"
        );
        std::cout << "PASS repeated accessible/inaccessible targets max_update_ms="
                  << maximumUpdateMilliseconds << '\n';
    }

    void TestTargetApproachSlowsDown()
    {
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        RobotConfig robotConfig;
        robotConfig.speed = 6.0f;
        Robot robot(robotConfig, { -10.0f, 0.65f, 0.0f });
        Target target({ 2.0f, 0.6f, 0.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.3f;

        Check(
            navigation.Initialize(world, robot, target),
            "target approach test initializes navigation"
        );

        float farSpeed = 0.0f;
        float speedNearTarget = 0.0f;
        for (int frame = 0; frame < 240; ++frame)
        {
            const Vector2 command =
                navigation.CalculateDesiredVelocity(
                    robot, target, TEST_DT, navigationConfig, world
            );
            robot.SetVelocity(command);
            farSpeed = std::max(farSpeed, Speed(robot.GetVelocity()));
            const Vector3 position = robot.GetPosition();
            const float dx = target.GetPosition().x - position.x;
            const float dz = target.GetPosition().z - position.z;
            if (std::sqrt(dx * dx + dz * dz) <= 3.0f)
                speedNearTarget = Speed(robot.GetVelocity());
            robot.Update(TEST_DT);
            world.Update(TEST_DT);
            navigation.Update(
                world, robot, target, navigationConfig, TEST_DT);
            if (navigation.GetState() == NavigationState::TargetReached)
                break;
        }

        Check(
            farSpeed > 5.0f,
            "automatic navigation reaches full speed before the target"
        );
        Check(
            speedNearTarget > 0.0f && speedNearTarget < 5.9f,
            "automatic navigation reduces speed near the target"
        );
        if (navigation.GetState() == NavigationState::TargetReached)
        {
            robot.SetVelocity(navigation.CalculateDesiredVelocity(
                robot, target, TEST_DT, navigationConfig, world));
        }
        Check(
            navigation.GetState() == NavigationState::TargetReached &&
                Speed(robot.GetVelocity()) < 0.01f,
            "robot slows and stops on arrival"
        );
        std::cout << "PASS target approach slows down and stops\n";
    }

    void TestLocalPlannerUsesUpdatedSpeedLimit()
    {
        const TerrainConfig terrain = MakeTerrain();
        GridMap globalGrid(terrain);
        World world(terrain);
        LocalGrid localGrid;
        localGrid.Initialize(40, 40, 1.0f);
        localGrid.UpdateWindow(0.0f, 0.0f);
        localGrid.UpdateFromGlobalGrid(globalGrid);
        LocalPlanner planner;

        for (const float speedLimit : { 2.0f, 4.0f, 6.0f, 8.0f })
        {
            planner.Configure(speedLimit, 0.65f);
            const Vector2 command = planner.CalculateVelocity(
                { 0.0f, 0.65f, 0.0f },
                { speedLimit, 0.0f },
                { 30.0f, 0.6f, 0.0f },
                {},
                globalGrid,
                TEST_DT,
                localGrid,
                world
            );
            const float measuredSpeed = Speed(command);
            std::cout << "LOCAL_SPEED limit=" << speedLimit
                      << " measured=" << measuredSpeed << " m/s\n";
            Check(
                measuredSpeed >= speedLimit * 0.95f &&
                    measuredSpeed <= speedLimit + 0.01f,
                "local planner immediately uses each updated speed limit"
            );
        }

        std::cout << "PASS local planner receives updated speed limits\n";
    }

    void TestStaticObstacleRemainsSafe()
    {
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        Obstacle obstacle;
        obstacle.position = { 0.0f, 0.5f, 0.0f };
        obstacle.size = { 2.0f, 1.0f, 5.0f };
        Check(world.AddObstacle(obstacle) != 0,
            "static obstacle registers for navigation test");

        RobotConfig robotConfig;
        robotConfig.speed = 4.0f;
        Robot robot(robotConfig, { -12.0f, 0.65f, 0.0f });
        Target target({ 12.0f, 0.6f, 0.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        navigationConfig.arrivalRadius = 0.2f;

        Check(
            navigation.Initialize(world, robot, target),
            "static-obstacle navigation initializes"
        );

        const Obstacle& staticObstacle = world.GetObstacles().front();
        for (int frame = 0; frame < 600; ++frame)
        {
            const Vector2 command =
                navigation.CalculateDesiredVelocity(
                    robot,
                    target,
                    TEST_DT,
                    navigationConfig,
                    world
                );
            robot.SetVelocity(command);
            robot.Update(TEST_DT);
            world.Update(TEST_DT);
            navigation.Update(
                world, robot, target, navigationConfig, TEST_DT);

            Check(
                !CollisionChecker::CircleIntersectsObstacle(
                    robot.GetPosition(),
                    robot.GetRadius(),
                    staticObstacle
                ),
                "automatic path following stays clear of a static obstacle"
            );
        }

        std::cout << "PASS static obstacle remains collision-free\n";
    }

    void TestStaticPassagesAndNarrowGap()
    {
        const TerrainConfig terrain = MakeTerrain();

        World leftPassageWorld(terrain);
        Obstacle leftBlocker;
        leftBlocker.position = { 0.0f, 0.5f, 0.0f };
        leftBlocker.size = { 2.0f, 1.0f, 6.0f };
        Check(leftPassageWorld.AddObstacle(leftBlocker) != 0,
            "left-passage obstacle registers");
        Obstacle rightWall;
        rightWall.position = { 0.0f, 0.5f, 19.0f };
        rightWall.size = { 4.0f, 1.0f, 38.0f };
        Check(leftPassageWorld.AddObstacle(rightWall) != 0,
            "right-side wall registers");
        const BehaviorMetrics left =
            RunBehaviorScenario(leftPassageWorld, 0.0f, 0.0f, 1000);
        ReportBehavior("static-left-passage", left);
        Check(left.initialized && left.reachedTarget && !left.collision,
            "robot finds and safely traverses the available left passage");
        Check(left.longestStoppedFrames < 60,
            "robot does not wait indefinitely at a static passage");

        World rightPassageWorld(terrain);
        Obstacle rightBlocker = leftBlocker;
        Check(rightPassageWorld.AddObstacle(rightBlocker) != 0,
            "right-passage obstacle registers");
        Obstacle leftWall;
        leftWall.position = { 0.0f, 0.5f, -19.0f };
        leftWall.size = { 4.0f, 1.0f, 38.0f };
        Check(rightPassageWorld.AddObstacle(leftWall) != 0,
            "left-side wall registers");
        const BehaviorMetrics right =
            RunBehaviorScenario(rightPassageWorld, 0.0f, 0.0f, 1000);
        ReportBehavior("static-right-passage", right);
        Check(right.initialized && right.reachedTarget && !right.collision,
            "robot finds and safely traverses the available right passage");
        Check(right.longestStoppedFrames < 60,
            "robot does not wait indefinitely at the right passage");

        World twoPassagesWorld(terrain);
        Check(twoPassagesWorld.AddObstacle(leftBlocker) != 0,
            "two-passage obstacle registers");
        const BehaviorMetrics both =
            RunBehaviorScenario(twoPassagesWorld, 0.0f, 0.0f, 1000);
        ReportBehavior("static-two-passages", both);
        Check(both.initialized && both.reachedTarget && !both.collision,
            "robot uses one of two safe passages around an obstacle");

        World narrowPassageWorld(terrain);
        for (const float z : { -2.8f, 2.8f })
        {
            Obstacle boundary;
            boundary.position = { 0.0f, 0.5f, z };
            boundary.size = { 2.0f, 1.0f, 2.0f };
            Check(narrowPassageWorld.AddObstacle(boundary) != 0,
                "narrow-passage boundary registers");
        }
        const BehaviorMetrics narrow =
            RunBehaviorScenario(narrowPassageWorld, 0.0f, 0.0f, 1000);
        ReportBehavior("static-narrow-passage", narrow);
        Check(narrow.initialized && narrow.reachedTarget && !narrow.collision,
            "robot safely negotiates a passage wider than its footprint");
    }

    void TestDynamicCrossingAndRearThreats()
    {
        const TerrainConfig terrain = MakeTerrain();

        World crossingWorld(terrain);
        Obstacle crossing;
        crossing.position = { -8.0f, 0.5f, -4.5f };
        crossing.size = { 1.5f, 1.0f, 2.0f };
        crossing.velocity = { 0.0f, 2.5f };
        crossing.isMoving = true;
        Check(crossingWorld.AddObstacle(crossing) != 0,
            "crossing dynamic obstacle registers");
        const BehaviorMetrics crossingMetrics =
            RunBehaviorScenario(crossingWorld, 0.0f, 0.0f, 1000);
        ReportBehavior("dynamic-crossing", crossingMetrics);
        Check(
            crossingMetrics.initialized &&
                !crossingMetrics.collision &&
                crossingMetrics.distanceTravelled > 15.0f,
            "robot avoids the crossing obstacle without stopping indefinitely"
        );

        World rearWorld(terrain);
        Obstacle rear;
        // Keep enough headway for a collision-free evasion to be feasible.
        rear.position = { -25.0f, 0.5f, 0.0f };
        rear.size = { 1.5f, 1.0f, 1.5f };
        rear.velocity = { 5.5f, 0.0f };
        rear.isMoving = true;
        Check(rearWorld.AddObstacle(rear) != 0,
            "rear dynamic obstacle registers");
        const BehaviorMetrics rearMetrics =
            RunBehaviorScenario(rearWorld, 0.0f, 0.0f, 1000);
        ReportBehavior("dynamic-rear-threat", rearMetrics);
        Check(
            rearMetrics.initialized &&
                !rearMetrics.collision &&
                rearMetrics.distanceTravelled > 15.0f &&
                rearMetrics.temporalSearches > 0 &&
                rearMetrics.longestStoppedFrames < 60,
            "robot responds safely to a catching obstacle from behind"
        );
    }

    void TestHeadOnThreatsAcrossSliderSpeeds()
    {
        const TerrainConfig terrain = MakeTerrain();
        const float speeds[] = { 2.0f, 4.0f, 6.0f, 8.0f };
        const float obstacleOffsets[] = { -0.3f, 0.0f, 0.2f, 0.4f };

        for (std::size_t i = 0; i < 4; ++i)
        {
            World world(terrain);
            Obstacle approaching;
            approaching.position = {
                -4.0f + obstacleOffsets[i],
                0.5f,
                0.0f
            };
            approaching.size = { 1.5f, 1.0f, 1.5f };
            approaching.velocity = {
                -3.0f + static_cast<float>(i) * 0.1f,
                0.0f
            };
            approaching.isMoving = true;
            Check(world.AddObstacle(approaching) != 0,
                "head-on obstacle registers");

            const BehaviorMetrics metrics =
                RunBehaviorScenario(
                    world,
                    0.0f,
                    0.0f,
                    1000,
                    0,
                    -1,
                    speeds[i]
                );
            ReportBehavior("head-on-slider-variation", metrics);
            Check(
                metrics.initialized &&
                    !metrics.collision &&
                    metrics.distanceTravelled > 8.0f &&
                    metrics.firstThreatFrame >= 0 &&
                    metrics.firstEvasiveFrame >= 0 &&
                    metrics.firstEvasiveFrame <=
                        metrics.firstThreatFrame + 30 &&
                    metrics.longestStoppedFrames < 60,
                "head-on threat produces a timely, collision-free evasion at every slider speed"
            );
            Check(
                metrics.lateralDirectionChanges <= 3,
                "head-on evasion does not oscillate repeatedly between sides"
            );
        }
    }

    void TestConvergingAndCrossingThreats()
    {
        const TerrainConfig terrain = MakeTerrain();
        World convergingWorld(terrain);
        Obstacle converging;
        converging.position = { -9.0f, 0.5f, 3.75f };
        converging.size = { 1.5f, 1.0f, 1.5f };
        converging.velocity = { 0.0f, -2.5f };
        converging.isMoving = true;
        Check(convergingWorld.AddObstacle(converging) != 0,
            "converging obstacle registers");
        const BehaviorMetrics convergingMetrics =
            RunBehaviorScenario(convergingWorld, 0.0f);
        ReportBehavior("converging-side-threat", convergingMetrics);
        Check(
            convergingMetrics.initialized &&
                !convergingMetrics.collision &&
                convergingMetrics.distanceTravelled > 8.0f &&
                convergingMetrics.firstThreatFrame >= 0,
            "robot avoids a predicted side convergence"
        );

        World twoSidedWorld(terrain);
        Obstacle fromBelow;
        fromBelow.position = { -9.0f, 0.5f, -3.75f };
        fromBelow.size = { 1.5f, 1.0f, 1.5f };
        fromBelow.velocity = { 0.0f, 2.5f };
        fromBelow.isMoving = true;
        Check(twoSidedWorld.AddObstacle(fromBelow) != 0,
            "lower converging obstacle registers");
        Obstacle fromAbove = fromBelow;
        fromAbove.position.z = 3.75f;
        fromAbove.velocity.y = -2.5f;
        Check(twoSidedWorld.AddObstacle(fromAbove) != 0,
            "upper converging obstacle registers");
        const BehaviorMetrics twoSidedMetrics =
            RunBehaviorScenario(twoSidedWorld, 0.0f, 0.0f, 1000, 0, -1, 6.0f);
        ReportBehavior("two-sided-convergence", twoSidedMetrics);
        Check(
            twoSidedMetrics.initialized &&
                !twoSidedMetrics.collision &&
                twoSidedMetrics.distanceTravelled > 8.0f &&
                twoSidedMetrics.firstThreatFrame >= 0 &&
                twoSidedMetrics.temporalSearches > 0,
            "robot handles multiple converging obstacles without collision"
        );
    }

    void TestObstaclePredictionReversesAtBoundary()
    {
        const TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        Obstacle bouncing;
        bouncing.position = { -4.0f, 0.5f, 37.5f };
        bouncing.size = { 1.5f, 1.0f, 1.5f };
        bouncing.velocity = { 0.0f, 6.0f };
        bouncing.isMoving = true;
        const std::uint32_t obstacleId = world.AddObstacle(bouncing);
        Check(obstacleId != 0,
            "boundary-reversing obstacle registers");

        RobotConfig robotConfig;
        robotConfig.speed = 4.0f;
        Robot robot(robotConfig, { -15.0f, 0.65f, 0.0f });
        Target target({ 15.0f, 0.6f, 0.0f });
        NavigationController navigation;
        NavigationConfig navigationConfig;
        Check(navigation.Initialize(world, robot, target),
            "boundary-reversal navigation initializes");

        bool observedDirectionChange = false;
        for (int frame = 0; frame < 12; ++frame)
        {
            const Vector2 command = navigation.CalculateDesiredVelocity(
                robot,
                target,
                TEST_DT,
                navigationConfig,
                world
            );
            robot.SetVelocity(command);
            robot.Update(TEST_DT);
            world.Update(TEST_DT);
            navigation.Update(
                world,
                robot,
                target,
                navigationConfig,
                TEST_DT
            );

            const LocalGrid* localGrid = navigation.GetLocalGrid();
            const auto& trajectories =
                localGrid->GetCachedObstacleTrajectories();
            const auto trajectory = std::find_if(
                trajectories.begin(),
                trajectories.end(),
                [obstacleId](const CachedObstacleTrajectory& candidate)
                {
                    return candidate.obstacleId == obstacleId;
                }
            );
            if (trajectory != trajectories.end() &&
                trajectory->samples.size() > 5)
            {
                observedDirectionChange =
                    trajectory->samples[1].position.z >
                        trajectory->samples[0].position.z &&
                    trajectory->samples[5].position.z <
                        trajectory->samples[4].position.z;
                if (observedDirectionChange)
                    break;
            }
        }

        Check(
            observedDirectionChange,
            "cached obstacle prediction includes a direction reversal at a terrain boundary"
        );
        std::cout << "PASS prediction cache tracks obstacle direction reversal\n";
    }

    void TestTemporaryBlockageAndNoSafeRoute()
    {
        const TerrainConfig terrain = MakeTerrain();

        World temporaryWorld(terrain);
        Obstacle temporary;
        temporary.position = { -1.0f, 0.5f, 0.0f };
        temporary.size = { 1.5f, 1.0f, 8.0f };
        temporary.velocity = { 0.0f, 0.0f };
        temporary.isMoving = true;
        const std::uint32_t temporaryId =
            temporaryWorld.AddObstacle(temporary);
        Check(temporaryId != 0,
            "temporary dynamic blockage registers");
        const BehaviorMetrics temporaryMetrics =
            RunBehaviorScenario(
                temporaryWorld, 0.0f, 0.0f, 1000, temporaryId, 120);
        ReportBehavior("temporary-blockage-removal", temporaryMetrics);
        Check(
            temporaryMetrics.initialized &&
                temporaryMetrics.reachedTarget &&
                !temporaryMetrics.collision,
            "robot resumes and reaches the target after blockage removal"
        );

        World impossibleWorld(terrain);
        Obstacle barrier;
        barrier.position = { 0.0f, 0.5f, 0.0f };
        barrier.size = { 3.0f, 1.0f, 80.0f };
        Check(impossibleWorld.AddObstacle(barrier) != 0,
            "uncrossable terrain-wide barrier registers");
        const BehaviorMetrics impossible =
            RunBehaviorScenario(impossibleWorld, 0.0f, 0.0f, 300);
        ReportBehavior("no-safe-passage", impossible);
        Check(
            !impossible.initialized &&
                !impossible.collision &&
                impossible.distanceTravelled < 0.01f,
            "robot does not force passage when no safe global route exists"
        );
    }
}

int main()
{
    TestManualSpeedLevels();
    TestAutomaticSpeedLevels();
    TestAutomaticSliderChangesDuringNavigation();
    TestMovingTargetReplansGlobalRoute();
    TestRepeatedRetargetsReplaceStalePaths();
    TestRetargetRecoversAfterInvalidDestination();
    TestRetargetDuringDynamicEvasion();
    TestRetargetWhileTemporalPathActive();
    TestNoGlobalPathWaitsAndRecovers();
    TestNoGlobalPathBehindImpassableWall();
    TestNoGlobalPathKeepsDynamicEvasion();
    TestRepeatedReachableAndUnreachableTargets();
    TestTargetApproachSlowsDown();
    TestLocalPlannerUsesUpdatedSpeedLimit();
    TestStaticObstacleRemainsSafe();
    TestStaticPassagesAndNarrowGap();
    TestDynamicCrossingAndRearThreats();
    TestHeadOnThreatsAcrossSliderSpeeds();
    TestConvergingAndCrossingThreats();
    TestObstaclePredictionReversesAtBoundary();
    TestTemporaryBlockageAndNoSafeRoute();
    std::cout << "All robot motion tests passed.\n";
    return EXIT_SUCCESS;
}
