#include "monde/EvilAhrEvent.h"
#include "navigation/LocalGrid.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace
{
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

    ObstacleGenerationConfig MakeObstacleSettings()
    {
        ObstacleGenerationConfig settings;
        settings.minSize = 1.0f;
        settings.maxSize = 1.0f;
        settings.minSpeed = 1.0f;
        settings.maxSpeed = 1.0f;
        return settings;
    }

    struct Scenario
    {
        World world{ MakeTerrain() };
        Robot robot{ RobotConfig{}, { -15.0f, 0.65f, 0.0f } };
        Target target{ { 15.0f, 0.6f, 0.0f } };
        NavigationController navigation;
        NavigationConfig navigationConfig;
        ObstacleGenerationConfig obstacleSettings =
            MakeObstacleSettings();

        Scenario()
        {
            navigationConfig.arrivalRadius = 0.3f;
            Check(
                navigation.Initialize(world, robot, target),
                "scenario navigation initializes"
            );
        }
    };

    void TestPathTargetedWaveAndObstacleLimit()
    {
        Scenario scenario;
        Obstacle baseObstacle;
        baseObstacle.position = { -35.0f, 0.5f, 35.0f };
        baseObstacle.size = { 1.0f, 1.0f, 1.0f };
        const std::uint32_t baseId =
            scenario.world.AddObstacle(baseObstacle);
        Check(baseId != 0, "pre-existing obstacle added");

        EvilAhrEvent event(42);
        EvilAhrConfig& config = event.GetConfig();
        config.annoyancePercent = 100.0f;
        config.maximumActiveObstacles = 2;
        config.movingObstaclePercent = 0.0f;
        config.lifetimeSeconds = 10.0f;

        event.RequestWave();
        event.Update(
            scenario.world,
            scenario.robot,
            scenario.target,
            scenario.navigation,
            scenario.obstacleSettings,
            0.0f
        );

        Check(
            scenario.world.GetObstacles().size() == 2,
            "wave respects total active obstacle cap"
        );
        Check(
            event.GetDiagnostics().lastWaveSpawned == 1,
            "wave fills only the remaining available obstacle slot"
        );

        const GridMap* grid = scenario.navigation.GetGlobalGrid();
        Check(grid != nullptr, "global grid is available");
        const Obstacle& evilObstacle =
            scenario.world.GetObstacles().back();
        float nearestPathDistance =
            std::numeric_limits<float>::infinity();
        for (const GridCell& cell : scenario.navigation.GetGlobalPath())
        {
            const Vector2 pathPosition = grid->CellToWorld(cell);
            const float dx = pathPosition.x - evilObstacle.position.x;
            const float dz = pathPosition.y - evilObstacle.position.z;
            nearestPathDistance = std::min(
                nearestPathDistance,
                std::sqrt(dx * dx + dz * dz)
            );
        }
        Check(
            nearestPathDistance < 2.0f,
            "high-intensity obstacle is targeted near the current global path"
        );

        event.RequestWave();
        event.Update(
            scenario.world,
            scenario.robot,
            scenario.target,
            scenario.navigation,
            scenario.obstacleSettings,
            0.0f
        );
        Check(
            scenario.world.GetObstacles().size() == 2,
            "event does not exceed configured active obstacle limit"
        );

        event.Reset(scenario.world);
        Check(
            scenario.world.GetObstacles().size() == 1 &&
                scenario.world.GetObstacles().front().id == baseId,
            "reset removes only Evil Ahr-owned obstacles"
        );
    }

    void TestStaticObstacleSynchronizesAndExpires()
    {
        Scenario scenario;
        const std::vector<GridCell> originalPath =
            scenario.navigation.GetGlobalPath();
        EvilAhrEvent event(73);
        EvilAhrConfig& config = event.GetConfig();
        config.annoyancePercent = 100.0f;
        config.maximumActiveObstacles = 10;
        config.movingObstaclePercent = 0.0f;
        config.lifetimeSeconds = 0.25f;

        event.RequestWave();
        event.Update(
            scenario.world,
            scenario.robot,
            scenario.target,
            scenario.navigation,
            scenario.obstacleSettings,
            0.0f
        );
        Check(
            event.GetDiagnostics().lastWaveSpawned > 0,
            "manual static wave generates an obstacle"
        );

        GridCell obstacleCell;
        const Obstacle& spawnedObstacle =
            scenario.world.GetObstacles().front();
        Check(
            scenario.navigation.GetGlobalGrid()->WorldToCell(
                spawnedObstacle.position.x,
                spawnedObstacle.position.z,
                obstacleCell),
            "spawned static obstacle maps to a global cell"
        );
        Check(
            std::any_of(
                originalPath.begin(),
                originalPath.end(),
                [&](const GridCell& cell)
                {
                    return cell.x == obstacleCell.x &&
                        cell.z == obstacleCell.z;
                }),
            "static obstacle occupies the previous global route"
        );

        scenario.navigation.Update(
            scenario.world,
            scenario.robot,
            scenario.target,
            scenario.navigationConfig,
            0.01f
        );
        Check(
            !scenario.navigation.GetGlobalPath().empty(),
            "D* Lite retains or finds a route after obstacle insertion"
        );
        Check(
            !scenario.navigation.GetGlobalGrid()->IsTraversable(
                obstacleCell),
            "GridSynchronizer marks the newly occupied route cell blocked"
        );
        Check(
            std::none_of(
                scenario.navigation.GetGlobalPath().begin(),
                scenario.navigation.GetGlobalPath().end(),
                [&](const GridCell& cell)
                {
                    return cell.x == obstacleCell.x &&
                        cell.z == obstacleCell.z;
                }),
            "D* Lite replans around the newly blocked cell"
        );

        event.Update(
            scenario.world,
            scenario.robot,
            scenario.target,
            scenario.navigation,
            scenario.obstacleSettings,
            0.3f
        );
        Check(
            scenario.world.GetObstacles().empty(),
            "owned obstacle is removed after its lifetime"
        );

        scenario.navigation.Update(
            scenario.world,
            scenario.robot,
            scenario.target,
            scenario.navigationConfig,
            0.05f
        );
        Check(
            !scenario.navigation.GetGlobalPath().empty(),
            "D* Lite restores a route after obstacle removal"
        );
        Check(
            scenario.navigation.GetGlobalGrid()->IsTraversable(
                obstacleCell),
            "GridSynchronizer frees the cell after obstacle removal"
        );
    }

    void TestDynamicObstaclePredictionIsRemoved()
    {
        Scenario scenario;
        EvilAhrEvent event(19);
        EvilAhrConfig& config = event.GetConfig();
        config.annoyancePercent = 100.0f;
        config.maximumActiveObstacles = 10;
        config.movingObstaclePercent = 100.0f;
        config.lifetimeSeconds = 0.25f;

        event.RequestWave();
        event.Update(
            scenario.world,
            scenario.robot,
            scenario.target,
            scenario.navigation,
            scenario.obstacleSettings,
            0.0f
        );
        Check(
            !scenario.world.GetObstacles().empty() &&
                scenario.world.GetObstacles().back().isMoving,
            "dynamic wave creates a moving obstacle"
        );
        const std::uint32_t dynamicId =
            scenario.world.GetObstacles().back().id;

        scenario.navigation.Update(
            scenario.world,
            scenario.robot,
            scenario.target,
            scenario.navigationConfig,
            0.05f
        );
        const LocalGrid* localGrid = scenario.navigation.GetLocalGrid();
        Check(localGrid != nullptr, "local grid is available");
        bool predictionFound = false;
        for (const CachedObstacleTrajectory& trajectory :
             localGrid->GetCachedObstacleTrajectories())
        {
            predictionFound =
                predictionFound || trajectory.obstacleId == dynamicId;
        }
        Check(predictionFound, "dynamic obstacle prediction is cached");

        event.Update(
            scenario.world,
            scenario.robot,
            scenario.target,
            scenario.navigation,
            scenario.obstacleSettings,
            0.3f
        );
        Check(
            scenario.world.GetObstacles().empty(),
            "expired dynamic obstacle is removed from the world"
        );

        scenario.navigation.Update(
            scenario.world,
            scenario.robot,
            scenario.target,
            scenario.navigationConfig,
            0.01f
        );
        for (const CachedObstacleTrajectory& trajectory :
             localGrid->GetCachedObstacleTrajectories())
        {
            Check(
                trajectory.obstacleId != dynamicId,
                "removed dynamic obstacle prediction is invalidated"
            );
        }
    }

    void TestDifficultyChangesWaveSizeAndFrequency()
    {
        Scenario lowScenario;
        Scenario highScenario;
        EvilAhrEvent lowEvent(123);
        EvilAhrEvent highEvent(123);
        lowEvent.GetConfig().annoyancePercent = 0.0f;
        highEvent.GetConfig().annoyancePercent = 100.0f;
        lowEvent.GetConfig().maximumActiveObstacles = 20;
        highEvent.GetConfig().maximumActiveObstacles = 20;
        lowEvent.GetConfig().movingObstaclePercent = 0.0f;
        highEvent.GetConfig().movingObstaclePercent = 0.0f;

        lowEvent.RequestWave();
        highEvent.RequestWave();
        lowEvent.Update(
            lowScenario.world,
            lowScenario.robot,
            lowScenario.target,
            lowScenario.navigation,
            lowScenario.obstacleSettings,
            0.0f
        );
        highEvent.Update(
            highScenario.world,
            highScenario.robot,
            highScenario.target,
            highScenario.navigation,
            highScenario.obstacleSettings,
            0.0f
        );
        Check(
            highEvent.GetDiagnostics().lastWaveSpawned >
                lowEvent.GetDiagnostics().lastWaveSpawned,
            "high difficulty produces larger multi-obstacle waves"
        );

        Scenario timedScenario;
        EvilAhrEvent timedEvent(456);
        EvilAhrConfig& timedConfig = timedEvent.GetConfig();
        timedConfig.enabled = true;
        timedConfig.annoyancePercent = 100.0f;
        timedConfig.spawnFrequencySeconds = 0.5f;
        timedConfig.maximumActiveObstacles = 10;
        timedConfig.movingObstaclePercent = 0.0f;

        timedEvent.Update(
            timedScenario.world,
            timedScenario.robot,
            timedScenario.target,
            timedScenario.navigation,
            timedScenario.obstacleSettings,
            0.1f
        );
        Check(
            timedEvent.GetDiagnostics().wavesTriggered == 0,
            "periodic spawning waits until the scaled frequency elapses"
        );
        timedEvent.Update(
            timedScenario.world,
            timedScenario.robot,
            timedScenario.target,
            timedScenario.navigation,
            timedScenario.obstacleSettings,
            0.1f
        );
        Check(
            timedEvent.GetDiagnostics().wavesTriggered == 1,
            "high difficulty shortens the periodic spawn interval"
        );
    }

    void TestHardLimitAndMaximumPopulationUpdateCost()
    {
        Scenario scenario;
        Obstacle existingObstacle;
        existingObstacle.position = { -35.0f, 0.5f, 35.0f };
        existingObstacle.size = { 0.5f, 1.0f, 0.5f };
        for (int i = 0; i < 200; ++i)
        {
            Check(
                scenario.world.AddObstacle(existingObstacle) != 0,
                "populate world to its 200-obstacle cap"
            );
        }

        EvilAhrEvent event(9001);
        EvilAhrConfig& config = event.GetConfig();
        config.maximumActiveObstacles = 500;
        event.RequestWave();
        event.Update(
            scenario.world,
            scenario.robot,
            scenario.target,
            scenario.navigation,
            scenario.obstacleSettings,
            0.0f
        );
        Check(
            config.maximumActiveObstacles == 200 &&
                scenario.world.GetObstacles().size() == 200,
            "hard cap clamps settings and prevents the 201st obstacle"
        );

        constexpr int measuredFrames = 6000;
        const auto start = std::chrono::steady_clock::now();
        for (int frame = 0; frame < measuredFrames; ++frame)
        {
            event.Update(
                scenario.world,
                scenario.robot,
                scenario.target,
                scenario.navigation,
                scenario.obstacleSettings,
                1.0f / 60.0f
            );
        }
        const auto end = std::chrono::steady_clock::now();
        const double totalMilliseconds =
            std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "EVIL_AHR_BENCH active_world_obstacles=200"
                  << " updates=" << measuredFrames
                  << " average_update_us="
                  << totalMilliseconds * 1000.0 / measuredFrames
                  << " total_ms=" << totalMilliseconds << '\n';
    }
}

int main()
{
    TestPathTargetedWaveAndObstacleLimit();
    TestStaticObstacleSynchronizesAndExpires();
    TestDynamicObstaclePredictionIsRemoved();
    TestDifficultyChangesWaveSizeAndFrequency();
    TestHardLimitAndMaximumPopulationUpdateCost();
    std::cout << "All Evil Ahr event tests passed.\n";
    return 0;
}
