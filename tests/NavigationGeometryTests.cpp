#include "planification/GridMap.h"
#include "navigation/LocalGrid.h"
#include "navigation/LocalPlanner.h"
#include "navigation/PathFollower.h"
#include "monde/Monde.h"
#include "robot/Robot.h"
#include "cible/Cible.h"

#include <array>
#include <cstdlib>
#include <iostream>

namespace
{
    void Check(bool condition, const char* message)
    {
        if (condition)
            return;

        std::cerr << "FAILED: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

int main()
{
    TerrainConfig terrain;
    terrain.cellsX = 20;
    terrain.cellsZ = 20;
    terrain.cellSize = 1.0f;

    GridMap grid(terrain);

    Check(
        grid.UpdateStaticObstacle(1, 0.0f, 0.0f, 2.0f, 2.0f, 0.65f),
        "initial obstacle registration"
    );

    Check(
        !grid.IsStaticPositionFree(1.4f, 1.4f, 0.65f),
        "circle intersects the rounded rectangle corner"
    );
    Check(
        grid.IsStaticPositionFree(1.5f, 1.5f, 0.65f),
        "clearance outside the rounded rectangle corner remains free"
    );
    Check(
        !grid.IsStaticMovementFree(-3.0f, 0.0f, 3.0f, 0.0f, 0.65f),
        "movement crossing an obstacle is rejected"
    );
    Check(
        !grid.IsStaticMovementFree(-3.0f, 1.6f, 3.0f, 1.6f, 0.65f),
        "swept circle collides with the rectangle side"
    );
    Check(
        grid.IsStaticMovementFree(-3.0f, 1.7f, 3.0f, 1.7f, 0.65f),
        "swept circle clears the rectangle side"
    );

    Check(
        grid.UpdateStaticObstacle(2, 0.0f, 0.0f, 2.0f, 2.0f, 0.65f),
        "overlapping obstacle registration"
    );
    Check(grid.RemoveStaticObstacle(1), "first overlapping obstacle removal");
    Check(
        !grid.IsStaticPositionFree(0.0f, 0.0f, 0.1f),
        "second obstacle remains indexed after first removal"
    );
    Check(grid.RemoveStaticObstacle(2), "second overlapping obstacle removal");
    Check(
        grid.IsStaticPositionFree(0.0f, 0.0f, 0.1f),
        "position clears after all overlapping obstacles are removed"
    );

    Check(
        grid.UpdateStaticObstacle(3, -4.0f, 0.0f, 1.0f, 1.0f, 0.65f),
        "obstacle registration before move"
    );
    Check(
        grid.UpdateStaticObstacle(3, 4.0f, 0.0f, 1.0f, 1.0f, 0.65f),
        "obstacle move update"
    );
    Check(
        grid.IsStaticPositionFree(-4.0f, 0.0f, 0.1f) &&
            !grid.IsStaticPositionFree(4.0f, 0.0f, 0.1f),
        "moved geometry replaces the previous position"
    );

    grid.Clear();
    Check(
        grid.IsStaticPositionFree(4.0f, 0.0f, 0.1f),
        "Clear removes geometry and its spatial index"
    );

    Check(
        grid.UpdateStaticObstacle(4, 4.0f, 0.0f, 1.0f, 1.0f, 0.65f),
        "obstacle registration before rebuild"
    );
    grid.Rebuild(terrain);
    Check(
        grid.IsStaticPositionFree(4.0f, 0.0f, 0.1f),
        "Rebuild removes geometry and its spatial index"
    );

    TerrainConfig offsetTerrain = terrain;
    offsetTerrain.cellsX = 21;
    offsetTerrain.cellsZ = 21;
    GridMap offsetGrid(offsetTerrain);

    LocalGrid localGrid;
    localGrid.Initialize(4, 4, offsetGrid.GetCellSize());
    localGrid.UpdateFromGlobalGrid(offsetGrid);

    const GridCell changedCell = { 10, 10 };
    Check(
        offsetGrid.SetCellState(changedCell, CellState::Blocked),
        "change a global cell on an offset grid"
    );
    localGrid.UpdateFromGlobalChanges(
        offsetGrid,
        offsetGrid.GetChangedCells()
    );

    Check(
        localGrid.GetStaticCellState({ 1, 1 }) == CellState::Blocked,
        "incremental sync updates the local sample of the changed global cell"
    );
    Check(
        localGrid.GetStaticCellState({ 2, 2 }) == CellState::Free,
        "incremental sync does not write the change to a neighboring sample"
    );

    World dynamicWorld(terrain);
    Obstacle movingObstacle;
    movingObstacle.position = { 0.0f, 0.5f, 0.0f };
    movingObstacle.size = { 1.0f, 1.0f, 1.0f };
    movingObstacle.velocity = { 0.5f, 0.0f };
    movingObstacle.isMoving = true;
    const std::uint32_t movingObstacleId =
        dynamicWorld.AddObstacle(movingObstacle);
    Check(movingObstacleId != 0, "dynamic obstacle registration");

    LocalGrid dynamicGrid;
    dynamicGrid.Initialize(40, 40, terrain.cellSize);
    Check(
        dynamicGrid.UpdateDynamicObstacles(
            dynamicWorld, 0.08f, 3.0f, 0.65f),
        "build prediction cache for a relevant obstacle"
    );
    Check(
        dynamicGrid.ConsumeRelevantObstacleSetChanged(),
        "newly relevant obstacle signals the controller"
    );
    Check(
        dynamicGrid.GetRelevantObstacleIds().size() == 1,
        "relevant obstacle ID is cached"
    );

    dynamicWorld.RemoveObstacle(movingObstacleId);
    Check(
        dynamicGrid.UpdateDynamicObstacles(
            dynamicWorld, 0.08f, 3.0f, 0.65f),
        "refresh prediction cache after obstacle removal"
    );
    Check(
        dynamicGrid.ConsumeRelevantObstacleSetChanged() &&
            dynamicGrid.GetRelevantObstacleIds().empty() &&
            dynamicGrid.GetCachedObstacleTrajectories().empty(),
        "removed obstacle leaves no stale ID or trajectory"
    );

    World bounceWorld(terrain);
    Obstacle bouncingObstacle;
    bouncingObstacle.position = { 9.5f, 0.5f, 0.0f };
    bouncingObstacle.size = { 1.0f, 1.0f, 1.0f };
    bouncingObstacle.velocity = { 1.0f, 0.0f };
    bouncingObstacle.isMoving = true;
    Check(
        bounceWorld.AddObstacle(bouncingObstacle) != 0,
        "register obstacle at terrain boundary"
    );
    const Obstacle predictedBounce =
        bounceWorld.GetPredictedObstacleState(
            bounceWorld.GetObstacles().front(),
            0.1f
        );
    bounceWorld.Update(0.1f);
    const Obstacle& actualBounce = bounceWorld.GetObstacles().front();
    Check(
        std::abs(predictedBounce.position.x - actualBounce.position.x) <
            0.0001f &&
        std::abs(predictedBounce.velocity.x - actualBounce.velocity.x) <
            0.0001f,
        "predicted direction change matches world boundary reflection"
    );

    for (const int dimension : std::array<int, 3>{ 400, 1000, 1500 })
    {
        TerrainConfig largeTerrain = terrain;
        largeTerrain.cellsX = dimension;
        largeTerrain.cellsZ = dimension;

        GridMap largeGrid(largeTerrain);
        Check(
            largeGrid.UpdateStaticObstacle(
                1, 0.0f, 0.0f, 2.0f, 2.0f, 0.65f),
            "large grid obstacle synchronization"
        );
        Check(
            !largeGrid.IsStaticPositionFree(0.0f, 0.0f, 0.65f),
            "large grid indexed collision query"
        );
    }

    GridMap speedGrid(terrain);
    LocalGrid speedLocalGrid;
    speedLocalGrid.Initialize(40, 40, terrain.cellSize);
    speedLocalGrid.UpdateFromGlobalGrid(speedGrid);

    RobotConfig robotConfig;
    robotConfig.speed = 3.0f;
    robotConfig.radius = 0.65f;
    const Vector2 startPosition = speedGrid.CellToWorld({ 3, 10 });
    const Vector2 targetPosition = speedGrid.CellToWorld({ 16, 10 });
    Robot speedRobot(
        robotConfig,
        { startPosition.x, 0.65f, startPosition.y }
    );
    Target speedTarget({ targetPosition.x, 0.6f, targetPosition.y });
    World speedWorld(terrain);
    LocalPlanner speedPlanner;
    speedPlanner.Configure(robotConfig.speed, robotConfig.radius);
    PathFollower speedFollower;
    std::vector<GridCell> speedPath;
    for (int x = 3; x <= 16; ++x)
        speedPath.push_back({ x, 10 });

    constexpr float speedTestDt = 1.0f / 60.0f;
    float requestedSpeedAtStart = 0.0f;
    float measuredSpeedAtOneSecond = 0.0f;
    float maximumMeasuredSpeed = 0.0f;

    for (int frame = 0; frame < 120; ++frame)
    {
        const Vector2 desired = speedFollower.CalculateDesiredVelocity(
            speedRobot,
            speedTarget,
            speedGrid,
            speedPath,
            0.1f
        );
        const float desiredSpeed = std::sqrt(
            desired.x * desired.x + desired.y * desired.y
        );
        if (frame == 0)
            requestedSpeedAtStart = desiredSpeed;

        const Vector2 command = speedPlanner.CalculateVelocity(
            speedRobot.GetPosition(),
            speedRobot.GetVelocity(),
            desired,
            speedTarget.GetPosition(),
            speedPath,
            speedGrid,
            speedTestDt,
            speedLocalGrid,
            speedWorld
        );
        speedRobot.SetVelocity(command);
        speedRobot.Update(speedTestDt);

        const Vector2 actualVelocity = speedRobot.GetVelocity();
        const float actualSpeed = std::sqrt(
            actualVelocity.x * actualVelocity.x +
            actualVelocity.y * actualVelocity.y
        );
        maximumMeasuredSpeed = std::max(maximumMeasuredSpeed, actualSpeed);
        if (frame == 59)
            measuredSpeedAtOneSecond = actualSpeed;
    }

    std::cout << "SPEED_TEST no_obstacle desired_initial="
              << requestedSpeedAtStart
              << " applied_at_1s=" << measuredSpeedAtOneSecond
              << " max_applied=" << maximumMeasuredSpeed << '\n';
    Check(
        requestedSpeedAtStart >= robotConfig.speed - 0.01f,
        "PathFollower requests maximum speed when the target is far"
    );
    Check(
        measuredSpeedAtOneSecond > 2.5f,
        "LocalPlanner accelerates normally on an unobstructed route"
    );

    std::cout << "All navigation geometry tests passed.\n";
    return EXIT_SUCCESS;
}