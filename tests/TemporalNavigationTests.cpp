#include "navigation/NavigationController.h"
#include "monde/Monde.h"
#include "robot/Robot.h"
#include "cible/Cible.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace
{
    constexpr float TEST_DT = 0.05f;

    void Check(bool condition, const char* message)
    {
        if (condition)
            return;

        std::cerr << "FAILED: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }

    TerrainConfig MakeTerrain()
    {
        TerrainConfig terrain;
        terrain.cellsX = 80;
        terrain.cellsZ = 80;
        terrain.cellSize = 1.0f;
        return terrain;
    }

    RobotConfig MakeRobotConfig()
    {
        RobotConfig config;
        config.radius = 0.65f;
        config.speed = 2.0f;
        return config;
    }

    struct NavigationFixture
    {
        TerrainConfig terrain;
        World world;
        Robot robot;
        Target target;
        NavigationController navigation;
        NavigationConfig navigationConfig;
        bool initialized = false;

        explicit NavigationFixture(
            float robotX = -10.0f,
            float targetX = 10.0f
        )
            : terrain(MakeTerrain()),
              world(terrain),
              robot(
                  MakeRobotConfig(),
                  { robotX, 0.65f, 0.0f }),
              target({ targetX, 0.6f, 0.0f })
        {
            navigationConfig.arrivalRadius = 0.2f;
        }

        std::uint32_t AddMovingObstacle(
            float x,
            float z,
            float velocityX,
            float velocityZ,
            float size = 1.0f
        )
        {
            Obstacle obstacle;
            obstacle.position = { x, size * 0.5f, z };
            obstacle.size = { size, size, size };
            obstacle.velocity = { velocityX, velocityZ };
            obstacle.isMoving = true;
            return world.AddObstacle(obstacle);
        }

        void Initialize()
        {
            initialized = navigation.Initialize(
                world,
                robot,
                target
            );
            Check(initialized, "navigation fixture initializes");
        }

        void Step(int frames)
        {
            for (int i = 0; i < frames; ++i)
            {
                navigation.Update(
                    world,
                    robot,
                    target,
                    navigationConfig,
                    TEST_DT
                );

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
            }
        }

        const TemporalPlanningDiagnostics& Diagnostics() const
        {
            return navigation.GetTemporalDiagnostics();
        }
    };

    struct TemporalSearchResult
    {
        bool succeeded = false;
        std::string failureReason;
        TemporalAStarSearchDiagnostics diagnostics;
        std::vector<TemporalWaypoint> path;
    };

    TemporalSearchResult RunTemporalSearch(
        World& world,
        float startX,
        float startZ,
        const std::vector<Vector2>& goalPositions,
        const std::vector<GridCell>& blockedCells = {},
        int localWidth = 40,
        float planningSpeed = 1.8f
    )
    {
        GridMap globalGrid(world.GetTerrain());
        LocalGrid localGrid;
        localGrid.Initialize(localWidth, localWidth, 1.0f);
        localGrid.UpdateWindow(startX, startZ);
        localGrid.UpdateFromGlobalGrid(globalGrid);
        Check(
            localGrid.UpdateDynamicObstacles(
                world, 0.08f, 3.0f, 0.65f),
            "direct test creates the obstacle prediction cache"
        );

        for (const GridCell& cell : blockedCells)
        {
            Check(
                localGrid.SetCellState(cell, CellState::Blocked),
                "direct test installs a static blocked cell"
            );
        }

        GridCell start;
        Check(
            localGrid.WorldToCell(startX, startZ, start),
            "direct test start lies in the local grid"
        );

        std::vector<GridCell> goals;
        for (const Vector2& position : goalPositions)
        {
            GridCell goal;
            if (localGrid.WorldToCell(
                position.x,
                position.y,
                goal))
            {
                goals.push_back(goal);
            }
        }

        TemporalAStar planner;
        TemporalAStarConfig config;
        config.timeStep = 0.1f;
        config.predictionHorizon = 3.0f;
        config.planningSpeed = planningSpeed;
        config.robotRadius = 0.65f;
        config.maxExpandedNodes = 4000;
        planner.Configure(config);

        TemporalSearchResult result;
        result.succeeded = planner.FindPath(
            localGrid,
            start,
            goals,
            result.path
        );
        result.failureReason = planner.GetLastFailureReason();
        result.diagnostics = planner.GetLastDiagnostics();
        return result;
    }

    void TestStaticObstacleDoesNotTriggerTemporalSearch()
    {
        NavigationFixture fixture;
        Obstacle obstacle;
        obstacle.position = { 0.0f, 0.5f, 0.0f };
        obstacle.size = { 3.0f, 1.0f, 3.0f };
        Check(fixture.world.AddObstacle(obstacle) != 0,
            "static obstacle is registered");
        fixture.Initialize();
        fixture.Step(20);

        Check(
            fixture.Diagnostics().searchesRun == 0,
            "static obstacle does not trigger Temporal A*"
        );
        Check(
            fixture.navigation.GetGlobalPath().size() > 2,
            "D* Lite retains a global path around the static obstacle"
        );
        std::cout << "PASS static obstacle uses global/local planning\n";
    }

    void TestMovingObstacleAhead()
    {
        NavigationFixture fixture;
        fixture.robot.SetVelocity({ 1.5f, 0.0f });
        fixture.AddMovingObstacle(-6.0f, 0.0f, -1.5f, 0.0f);
        fixture.Initialize();
        fixture.Step(12);

        Check(
            fixture.Diagnostics().threatsDetected > 0,
            "approaching obstacle ahead is detected"
        );
        Check(
            fixture.Diagnostics().searchesRun > 0,
            "approaching obstacle ahead triggers Temporal A*"
        );
        std::cout << "PASS moving obstacle ahead\n";
    }

    void TestRearObstacleCatchingRobot()
    {
        NavigationFixture fixture;
        fixture.robot.SetVelocity({ 2.0f, 0.0f });
        fixture.AddMovingObstacle(-13.5f, 0.0f, 3.0f, 0.0f);
        fixture.Initialize();
        fixture.Step(12);

        Check(
            fixture.Diagnostics().rearThreatsDetected > 0,
            "catching obstacle behind the robot is counted"
        );
        Check(
            fixture.Diagnostics().minimumTimeToCollision >= 0.0f &&
                fixture.Diagnostics().minimumTimeToCollision <= 2.5f,
            "rear threat exposes a finite TTC inside the reaction horizon"
        );
        Check(
            fixture.Diagnostics().searchesRun > 0,
            "rear threat triggers Temporal A*"
        );
        Check(
            fixture.Diagnostics().searchesSucceeded > 0,
            "Temporal A* finds a safe detour from a rear threat"
        );
        Check(
            fixture.Diagnostics().mostDangerousObstacleId != 0,
            "most dangerous rear obstacle ID is reported"
        );
        Check(
            fixture.Diagnostics().lastTriggerReason.find("arriere") !=
                std::string::npos,
            "rear threat trigger reason is reported"
        );
        Check(
            fixture.Diagnostics().searchesRun <= 3,
            "Temporal A* remains rate-limited"
        );
        Check(
            !fixture.Diagnostics().recentSearchDurationsMilliseconds.empty(),
            "duration is recorded for executed temporal searches"
        );
        std::cout << "PASS rear obstacle catches robot\n";
    }

    void TestLateralObstacle()
    {
        NavigationFixture fixture;
        fixture.robot.SetVelocity({ 1.5f, 0.0f });
        fixture.AddMovingObstacle(-8.0f, 3.0f, 0.0f, -3.0f);
        fixture.Initialize();
        bool observedThreat = false;
        for (int frame = 0; frame < 12; ++frame)
        {
            fixture.Step(1);
            observedThreat = observedThreat ||
                fixture.Diagnostics().threatsDetected > 0;
        }

        Check(
            observedThreat,
            "lateral obstacle converging on the robot is detected"
        );
        Check(
            fixture.Diagnostics().searchesRun > 0,
            "lateral threat triggers Temporal A*"
        );
        std::cout << "PASS lateral obstacle\n";
    }

    void TestTwoObstaclesConverging()
    {
        NavigationFixture fixture;
        fixture.robot.SetVelocity({ 1.0f, 0.0f });
        fixture.AddMovingObstacle(-8.0f, 2.0f, 0.0f, -2.0f);
        fixture.AddMovingObstacle(-8.0f, -2.0f, 0.0f, 2.0f);
        fixture.Initialize();
        int maximumDetectedThreats = 0;
        for (int frame = 0; frame < 12; ++frame)
        {
            fixture.Step(1);
            maximumDetectedThreats = std::max(
                maximumDetectedThreats,
                fixture.Diagnostics().threatsDetected
            );
        }

        Check(
            maximumDetectedThreats >= 2,
            "both obstacles in a converging pair are counted"
        );
        Check(
            fixture.Diagnostics().searchesRun > 0,
            "converging obstacles trigger Temporal A*"
        );
        std::cout << "PASS two converging obstacles\n";
    }

    void TestSeveralDynamicObstaclesInWindow()
    {
        NavigationFixture fixture;
        fixture.AddMovingObstacle(-8.0f, 0.0f, -1.0f, 0.0f);
        fixture.AddMovingObstacle(-4.0f, 3.0f, 0.0f, -2.0f);
        fixture.AddMovingObstacle(-12.0f, -4.0f, 0.0f, 2.0f);
        fixture.Initialize();
        fixture.Step(4);

        Check(
            fixture.navigation.GetLocalGrid()
                ->GetRelevantObstacleIds().size() >= 3,
            "all moving obstacles in the swept local window are registered"
        );
        std::cout << "PASS multiple dynamic obstacles in local window\n";
    }

    void TestObstacleDirectionChange()
    {
        NavigationFixture fixture(-35.0f, 0.0f);
        fixture.AddMovingObstacle(-38.5f, 0.0f, -4.0f, 0.0f);
        fixture.Initialize();
        fixture.Step(24);

        Check(
            fixture.Diagnostics().threatsDetected > 0,
            "predicted boundary reflection reveals the returning obstacle"
        );
        Check(
            fixture.Diagnostics().searchesRun > 0,
            "direction-changing threat triggers Temporal A*"
        );
        std::cout << "PASS obstacle direction change\n";
    }

    void TestNoSafeTemporalPathStopsRobot()
    {
        NavigationFixture fixture;
        fixture.AddMovingObstacle(-10.0f, 0.0f, 0.0f, 0.0f, 45.0f);
        fixture.Initialize();
        fixture.Step(12);

        Check(
            fixture.Diagnostics().searchesRun == 0 &&
                fixture.Diagnostics().lastSearchFailureReason.find(
                    "Aucun objectif temporel libre") != std::string::npos,
            "an impossible dynamic corridor is rejected before an unsafe search"
        );
        const Vector2 command =
            fixture.navigation.CalculateDesiredVelocity(
                fixture.robot,
                fixture.target,
                TEST_DT,
                fixture.navigationConfig,
                fixture.world
            );
        Check(
            std::abs(command.x) < 0.001f &&
                std::abs(command.y) < 0.001f,
            "the robot stops instead of following the unsafe global path"
        );
        std::cout << "PASS no safe temporal path stops robot\n";
    }

    void TestObstacleDisappearsAndGlobalPathResumes()
    {
        NavigationFixture fixture;
        fixture.robot.SetVelocity({ 2.0f, 0.0f });
        const std::uint32_t obstacleId =
            fixture.AddMovingObstacle(-13.5f, 0.0f, 3.0f, 0.0f);
        fixture.Initialize();
        fixture.Step(12);

        Check(
            fixture.Diagnostics().searchesRun > 0,
            "a diversion search ran before obstacle removal"
        );
        Check(
            fixture.Diagnostics().searchesSucceeded > 0,
            "Temporal A* found a diversion before obstacle removal"
        );
        Check(
            fixture.world.RemoveObstacle(obstacleId),
            "threatening obstacle is removed"
        );
        fixture.Step(8);

        Check(
            fixture.Diagnostics().threatsDetected == 0,
            "removed obstacle clears the threat assessment"
        );
        Check(
            !fixture.navigation.IsTemporalPathActive(),
            "temporal diversion is released after the danger disappears"
        );
        const Vector2 resumedCommand =
            fixture.navigation.CalculateDesiredVelocity(
                fixture.robot,
                fixture.target,
                TEST_DT,
                fixture.navigationConfig,
                fixture.world
            );
        Check(
            std::sqrt(
                resumedCommand.x * resumedCommand.x +
                resumedCommand.y * resumedCommand.y
            ) > 0.0f,
            "global path following resumes after danger removal"
        );
        std::cout << "PASS obstacle disappearance and global-path resumption\n";
    }

    void TestTemporalSearchFindsDetours()
    {
        TerrainConfig terrain = MakeTerrain();
        World frontWorld(terrain);
        Obstacle frontObstacle;
        frontObstacle.position = { 2.0f, 0.5f, 0.0f };
        frontObstacle.size = { 1.0f, 1.0f, 1.0f };
        frontObstacle.velocity = { 0.0f, 0.0f };
        frontObstacle.isMoving = true;
        Check(frontWorld.AddObstacle(frontObstacle) != 0,
            "front obstacle registered");
        const TemporalSearchResult front = RunTemporalSearch(
            frontWorld, 0.0f, 0.0f, { { 4.0f, 0.0f } });
        if (!front.succeeded)
        {
            std::cerr << "front failure=" << front.failureReason
                      << " expanded=" << front.diagnostics.expandedNodes
                      << " collision="
                      << front.diagnostics.rejectedCollisions
                      << " horizon="
                      << front.diagnostics.rejectedTimeHorizon
                      << " window="
                      << front.diagnostics.rejectedLocalWindow << '\n';
        }
        Check(front.succeeded, "A* routes around a mobile obstacle ahead");

        World rearWorld(terrain);
        Obstacle rearObstacle;
        rearObstacle.position = { -2.0f, 0.5f, 0.0f };
        rearObstacle.size = { 1.0f, 1.0f, 1.0f };
        rearObstacle.velocity = { 1.5f, 0.0f };
        rearObstacle.isMoving = true;
        Check(rearWorld.AddObstacle(rearObstacle) != 0,
            "rear obstacle registered");
        const TemporalSearchResult rear = RunTemporalSearch(
            rearWorld, 0.0f, 0.0f, { { 5.0f, 0.0f } });
        if (!rear.succeeded)
        {
            std::cerr << "rear failure=" << rear.failureReason
                      << " expanded=" << rear.diagnostics.expandedNodes
                      << " collision="
                      << rear.diagnostics.rejectedCollisions
                      << " horizon="
                      << rear.diagnostics.rejectedTimeHorizon
                      << " window="
                      << rear.diagnostics.rejectedLocalWindow << '\n';
        }
        Check(rear.succeeded, "A* routes away from a catching obstacle");

        World passageWorld(terrain);
        for (const float z : { -1.8f, 1.8f })
        {
            Obstacle obstacle;
            obstacle.position = { 2.5f, 0.5f, z };
            obstacle.size = { 1.5f, 1.0f, 1.0f };
            obstacle.isMoving = true;
            Check(passageWorld.AddObstacle(obstacle) != 0,
                "passage obstacle registered");
        }
        const TemporalSearchResult passage = RunTemporalSearch(
            passageWorld, 0.0f, 0.0f, { { 4.5f, 0.0f } });
        if (!passage.succeeded)
        {
            std::cerr << "passage failure=" << passage.failureReason
                      << " expanded="
                      << passage.diagnostics.expandedNodes
                      << " collision="
                      << passage.diagnostics.rejectedCollisions
                      << " horizon="
                      << passage.diagnostics.rejectedTimeHorizon
                      << " window="
                      << passage.diagnostics.rejectedLocalWindow << '\n';
        }
        Check(passage.succeeded, "A* finds the lateral opening between obstacles");

        std::cout << "PASS A* detours: ahead expanded="
                  << front.diagnostics.expandedNodes
                  << " rear expanded=" << rear.diagnostics.expandedNodes
                  << " passage expanded="
                  << passage.diagnostics.expandedNodes << '\n';
    }

    void TestTemporalSearchCanWait()
    {
        TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        Obstacle crossingObstacle;
        crossingObstacle.position = { 2.5f, 0.5f, 0.0f };
        crossingObstacle.size = { 1.5f, 1.0f, 7.0f };
        crossingObstacle.velocity = { 0.0f, 3.0f };
        crossingObstacle.isMoving = true;
        Check(world.AddObstacle(crossingObstacle) != 0,
            "crossing obstacle registered");

        std::vector<GridCell> corridorWalls;
        for (int x = 0; x < 40; ++x)
        {
            for (int z = 0; z < 40; ++z)
            {
                if (z <= 18 || z >= 22)
                    corridorWalls.push_back({ x, z });
            }
        }

        const TemporalSearchResult result = RunTemporalSearch(
            world,
            -1.0f,
            0.0f,
            { { 4.0f, 0.0f } },
            corridorWalls,
            40,
            4.0f);
        if (!result.succeeded)
        {
            std::cerr << "wait failure=" << result.failureReason
                      << " expanded=" << result.diagnostics.expandedNodes
                      << " collision="
                      << result.diagnostics.rejectedCollisions
                      << " horizon="
                      << result.diagnostics.rejectedTimeHorizon
                      << " window="
                      << result.diagnostics.rejectedLocalWindow << '\n';
        }
        Check(result.succeeded, "A* crosses after the moving barrier clears");

        bool containsWait = false;
        for (std::size_t i = 1; i < result.path.size(); ++i)
        {
            if (result.path[i].cell.x == result.path[i - 1].cell.x &&
                result.path[i].cell.z == result.path[i - 1].cell.z &&
                result.path[i].arrivalTime >
                    result.path[i - 1].arrivalTime)
            {
                containsWait = true;
                break;
            }
        }
        Check(containsWait, "the selected path contains an explicit wait");
        std::cout << "PASS temporal search waits for a crossing obstacle\n";
    }

    void TestBlockedGoalCanUseProgressingAlternative()
    {
        TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        GridMap globalGrid(terrain);
        LocalGrid localGrid;
        localGrid.Initialize(40, 40, 1.0f);
        localGrid.UpdateWindow(0.0f, 0.0f);
        localGrid.UpdateFromGlobalGrid(globalGrid);
        Check(
            localGrid.UpdateDynamicObstacles(world, 0.08f, 3.0f, 0.65f),
            "empty prediction cache is initialized"
        );

        GridCell start;
        GridCell blockedGoal;
        GridCell alternativeGoal;
        Check(localGrid.WorldToCell(0.0f, 0.0f, start),
            "goal-region start maps locally");
        Check(localGrid.WorldToCell(3.0f, 0.0f, blockedGoal),
            "blocked goal maps locally");
        Check(localGrid.WorldToCell(4.5f, 2.0f, alternativeGoal),
            "alternative goal maps locally");
        Check(localGrid.SetCellState(blockedGoal, CellState::Blocked),
            "initial exact goal is blocked");

        TemporalAStar planner;
        TemporalAStarConfig config;
        config.planningSpeed = 2.0f;
        config.robotRadius = 0.65f;
        planner.Configure(config);

        std::vector<TemporalWaypoint> path;
        Check(
            !planner.FindPath(
                localGrid, start, blockedGoal, path),
            "exact blocked goal reproduces goal unreachability"
        );
        Check(
            std::string(planner.GetLastFailureReason()) ==
                "GoalUnreachable",
            "blocked exact goal has a specific failure diagnosis"
        );

        Check(
            planner.FindPath(
                localGrid,
                start,
                std::vector<GridCell>{ blockedGoal, alternativeGoal },
                path),
            "goal region chooses an unblocked forward alternative"
        );
        Check(
            path.back().cell.x == alternativeGoal.x &&
                path.back().cell.z == alternativeGoal.z,
            "alternative goal preserves forward progress"
        );
        std::cout << "PASS blocked exact goal uses forward goal-region cell\n";
    }

    void TestDetourNearLocalWindowBoundary()
    {
        TerrainConfig terrain = MakeTerrain();
        World world(terrain);
        Obstacle obstacle;
        obstacle.position = { 2.5f, 0.5f, 0.0f };
        obstacle.size = { 2.0f, 1.0f, 2.0f };
        obstacle.isMoving = true;
        Check(world.AddObstacle(obstacle) != 0,
            "boundary test obstacle registered");

        const TemporalSearchResult result = RunTemporalSearch(
            world, 0.0f, 0.0f, { { 4.5f, 0.0f } }, {}, 16, 3.0f);
        if (!result.succeeded)
        {
            std::cerr << "boundary failure=" << result.failureReason
                      << " expanded=" << result.diagnostics.expandedNodes
                      << " collision="
                      << result.diagnostics.rejectedCollisions
                      << " horizon="
                      << result.diagnostics.rejectedTimeHorizon
                      << " window="
                      << result.diagnostics.rejectedLocalWindow << '\n';
        }
        Check(
            result.succeeded,
            "A* finds a safe detour before reaching the local window edge"
        );
        std::cout << "PASS detour near local window boundary expanded="
                  << result.diagnostics.expandedNodes << '\n';
    }

    void TestFailureDiagnosticsSeparateHorizonAndCollision()
    {
        TerrainConfig terrain = MakeTerrain();
        World clearWorld(terrain);
        const TemporalSearchResult horizon = RunTemporalSearch(
            clearWorld, 0.0f, 0.0f, { { 10.0f, 0.0f } });
        Check(
            !horizon.succeeded &&
                horizon.failureReason == "PredictionHorizonExceeded",
            "a route longer than the prediction horizon is identified"
        );
        Check(
            horizon.diagnostics.rejectedTimeHorizon > 0 &&
                horizon.diagnostics.expandedNodes > 0,
            "horizon failure reports node and time rejection counts"
        );

        World blockedWorld(terrain);
        Obstacle blockingObstacle;
        blockingObstacle.position = { 0.0f, 0.5f, 0.0f };
        blockingObstacle.size = { 30.0f, 1.0f, 30.0f };
        blockingObstacle.isMoving = true;
        Check(blockedWorld.AddObstacle(blockingObstacle) != 0,
            "blocking obstacle registered");
        const TemporalSearchResult blocked = RunTemporalSearch(
            blockedWorld, 0.0f, 0.0f, { { 4.0f, 0.0f } });
        Check(
            !blocked.succeeded &&
                blocked.failureReason == "NoValidSuccessor",
            "a start with no collision-free transition is identified"
        );
        Check(
            blocked.diagnostics.rejectedCollisions > 0 &&
                blocked.diagnostics.rejectedNeighbors > 0,
            "collision and neighbor rejection counts are reported"
        );
        std::cout << "PASS diagnostics horizon="
                  << horizon.diagnostics.rejectedTimeHorizon
                  << " nodes=" << horizon.diagnostics.expandedNodes
                  << " blocked-collisions="
                  << blocked.diagnostics.rejectedCollisions << '\n';
    }
}

int main()
{
    TestStaticObstacleDoesNotTriggerTemporalSearch();
    TestMovingObstacleAhead();
    TestRearObstacleCatchingRobot();
    TestLateralObstacle();
    TestTwoObstaclesConverging();
    TestSeveralDynamicObstaclesInWindow();
    TestObstacleDirectionChange();
    TestNoSafeTemporalPathStopsRobot();
    TestObstacleDisappearsAndGlobalPathResumes();
    TestTemporalSearchFindsDetours();
    TestTemporalSearchCanWait();
    TestBlockedGoalCanUseProgressingAlternative();
    TestDetourNearLocalWindowBoundary();
    TestFailureDiagnosticsSeparateHorizonAndCollision();

    std::cout << "All temporal navigation tests passed.\n";
    return EXIT_SUCCESS;
}
