
#include "TemporalAStar.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace
{
    constexpr float EPSILON = 0.00001f;

    struct SearchNode
    {
        std::size_t index = 0;
        float g = 0.0f;
        float f = 0.0f;
    };

    struct CompareNodes
    {
        bool operator()(
            const SearchNode& a,
            const SearchNode& b
        ) const
        {
            return a.f > b.f;
        }
    };

    bool SameCell(const GridCell& a, const GridCell& b)
    {
        return a.x == b.x && a.z == b.z;
    }

    float Heuristic(
        const GridCell& a,
        const std::vector<GridCell>& goals,
        float cellSize,
        float speed,
        float diagonalCost
    )
    {
        float minimumDistance =
            std::numeric_limits<float>::infinity();

        for (const GridCell& goal : goals)
        {
            const int dx = std::abs(a.x - goal.x);
            const int dz = std::abs(a.z - goal.z);
            const int diagonal = std::min(dx, dz);
            const int straight = std::max(dx, dz) - diagonal;
            const float distance =
                diagonal * cellSize * diagonalCost +
                straight * cellSize;
            minimumDistance = std::min(
                minimumDistance,
                distance
            );
        }

        return minimumDistance / speed;
    }

    bool GetObstacleAtTime(
        const CachedObstacleTrajectory& trajectory,
        float time,
        PredictedObstaclePoint& result
    )
    {
        const auto& samples = trajectory.samples;

        if (samples.empty())
            return false;

        if (time < samples.front().time - EPSILON ||
            time > samples.back().time + EPSILON)
        {
            return false;
        }

        auto upper = std::lower_bound(
            samples.begin(),
            samples.end(),
            time,
            [](const PredictedObstaclePoint& point, float t)
            {
                return point.time < t;
            }
        );

        if (upper == samples.begin())
        {
            result = *upper;
            return true;
        }

        if (upper == samples.end())
        {
            result = samples.back();
            return true;
        }

        const auto& b = *upper;
        const auto& a = *(upper - 1);

        const float duration = b.time - a.time;

        if (duration <= EPSILON)
        {
            result = b;
            return true;
        }

        const float alpha = std::clamp(
            (time - a.time) / duration,
            0.0f,
            1.0f
        );

        result.obstacleId = trajectory.obstacleId;

        result.position = {
            a.position.x + (b.position.x - a.position.x) * alpha,
            a.position.y + (b.position.y - a.position.y) * alpha,
            a.position.z + (b.position.z - a.position.z) * alpha
        };

        result.size = {
            a.size.x + (b.size.x - a.size.x) * alpha,
            a.size.y + (b.size.y - a.size.y) * alpha,
            a.size.z + (b.size.z - a.size.z) * alpha
        };

        result.time = time;

        return true;
    }

    enum class SegmentSafety
    {
        Safe,
        LocalWindowExceeded,
        StaticCollision,
        DynamicCollision,
        PredictionUnavailable
    };

    SegmentSafety IsDynamicPositionSafe(
        float worldX,
        float worldZ,
        float plannedTime,
        const LocalGrid& localGrid,
        const std::vector<CachedObstacleTrajectory>& trajectories,
        float robotRadius
    )
    {
        const float cacheTime =
            localGrid.GetPredictionCacheAge() + plannedTime;

        for (const auto& trajectory : trajectories)
        {
            PredictedObstaclePoint obstacle;

            if (!GetObstacleAtTime(
                trajectory,
                cacheTime,
                obstacle))
            {
                return SegmentSafety::PredictionUnavailable;
            }

            const float halfWidth =
                std::max(0.0f, obstacle.size.x * 0.5f);

            const float halfDepth =
                std::max(0.0f, obstacle.size.z * 0.5f);

            const float nearestX = std::clamp(
                worldX,
                obstacle.position.x - halfWidth,
                obstacle.position.x + halfWidth
            );

            const float nearestZ = std::clamp(
                worldZ,
                obstacle.position.z - halfDepth,
                obstacle.position.z + halfDepth
            );

            const float dx = worldX - nearestX;
            const float dz = worldZ - nearestZ;

            if (dx * dx + dz * dz <=
                robotRadius * robotRadius)
            {
                return SegmentSafety::DynamicCollision;
            }
        }

        return SegmentSafety::Safe;
    }

    SegmentSafety IsSegmentSafe(
        const LocalGrid& localGrid,
        const std::vector<CachedObstacleTrajectory>& trajectories,
        const Vector2& start,
        const Vector2& end,
        float startTime,
        float duration,
        float timeStep,
        float robotRadius
    )
    {
        const float dx = end.x - start.x;
        const float dz = end.y - start.y;

        const float distance = std::sqrt(dx * dx + dz * dz);

        const float spatialStep =
            std::max(0.1f, localGrid.GetCellSize() * 0.25f);

        const int spatialSamples = static_cast<int>(
            std::ceil(distance / spatialStep)
        );

        const int temporalSamples = static_cast<int>(
            std::ceil(duration / (timeStep * 0.5f))
        );

        const int samples = std::max(
            1,
            std::max(spatialSamples, temporalSamples)
        );

        for (int i = 0; i <= samples; ++i)
        {
            const float alpha =
                static_cast<float>(i) / samples;

            const float x = start.x + dx * alpha;
            const float z = start.y + dz * alpha;

            GridCell minCell;
            GridCell maxCell;
            if (!localGrid.WorldToCell(
                x - robotRadius,
                z - robotRadius,
                minCell) ||
                !localGrid.WorldToCell(
                x + robotRadius,
                z + robotRadius,
                maxCell))
            {
                return SegmentSafety::LocalWindowExceeded;
            }

            const float time =
                startTime + duration * alpha;

            // Vérification des obstacles statiques.
            if (!localGrid.IsStaticFootprintFree(
                x,
                z,
                robotRadius))
            {
                return SegmentSafety::StaticCollision;
            }

            // Vérification des trajectoires dynamiques.
            const SegmentSafety dynamicSafety =
                IsDynamicPositionSafe(
                x,
                z,
                time,
                localGrid,
                trajectories,
                robotRadius);
            if (dynamicSafety != SegmentSafety::Safe)
                return dynamicSafety;
        }

        return SegmentSafety::Safe;
    }
}

// ============================================
// CONFIGURATION
// ============================================

void TemporalAStar::Configure(
    const TemporalAStarConfig& newConfig
)
{
    if (std::isfinite(newConfig.timeStep) &&
        newConfig.timeStep > 0.0f)
    {
        config.timeStep = newConfig.timeStep;
    }

    if (std::isfinite(newConfig.predictionHorizon) &&
        newConfig.predictionHorizon > 0.0f)
    {
        config.predictionHorizon =
            newConfig.predictionHorizon;
    }

    if (std::isfinite(newConfig.planningSpeed) &&
        newConfig.planningSpeed > 0.0f)
    {
        config.planningSpeed = newConfig.planningSpeed;
    }

    if (newConfig.maxExpandedNodes > 0)
    {
        config.maxExpandedNodes =
            newConfig.maxExpandedNodes;
    }

    if (std::isfinite(newConfig.robotRadius) &&
        newConfig.robotRadius >= 0.0f)
    {
        config.robotRadius = newConfig.robotRadius;
    }

    if (std::isfinite(newConfig.waitCost) &&
        newConfig.waitCost >= 0.0f)
    {
        config.waitCost = newConfig.waitCost;
    }

    if (std::isfinite(newConfig.diagonalCost) &&
        newConfig.diagonalCost >= 1.0f)
    {
        config.diagonalCost = newConfig.diagonalCost;
    }
}

// ============================================
// TEMPORAL A*
// ============================================


bool TemporalAStar::FindPath(
    const LocalGrid& localGrid,
    const GridCell& start,
    const GridCell& goal,
    std::vector<TemporalWaypoint>& outputPath
) const
{
    return FindPath(
        localGrid,
        start,
        std::vector<GridCell>{ goal },
        outputPath
    );
}

bool TemporalAStar::FindPath(
    const LocalGrid& localGrid,
    const GridCell& start,
    const std::vector<GridCell>& acceptableGoals,
    std::vector<TemporalWaypoint>& outputPath
) const
{
    outputPath.clear();

    lastExpandedNodes = 0;
    lastDiagnostics = {};
    hasSearched = true;
    lastSearchSucceeded = false;
    lastFailureReason = FailureReason::None;

    // Enregistre la raison de l'échec.
    auto Fail = [&](FailureReason reason) -> bool
    {
        lastFailureReason = reason;
        return false;
    };

    if (!localGrid.IsInitialized())
        return Fail(FailureReason::GridNotInitialized);

    if (!std::isfinite(config.timeStep) ||
        config.timeStep <= 0.0f ||
        !std::isfinite(config.planningSpeed) ||
        config.planningSpeed <= 0.0f ||
        !std::isfinite(config.predictionHorizon) ||
        config.predictionHorizon <= 0.0f)
    {
        return Fail(FailureReason::InvalidConfig);
    }

    const int width = localGrid.GetWidth();
    const int height = localGrid.GetHeight();

    if (width <= 0 || height <= 0)
        return Fail(FailureReason::InvalidDimensions);

    if (start.x < 0 || start.z < 0 ||
        start.x >= width || start.z >= height)
    {
        return Fail(FailureReason::InvalidCoordinates);
    }

    if (localGrid.GetStaticCellState(start) != CellState::Free)
    {
        return Fail(FailureReason::StartBlocked);
    }

    const Vector2 startPosition =
        localGrid.CellToWorld(start);
    if (!localGrid.IsStaticFootprintFree(
        startPosition.x,
        startPosition.y,
        config.robotRadius))
    {
        return Fail(FailureReason::StartBlocked);
    }

    std::vector<GridCell> goals;
    goals.reserve(acceptableGoals.size());
    for (const GridCell& goal : acceptableGoals)
    {
        if (goal.x < 0 || goal.z < 0 ||
            goal.x >= width || goal.z >= height ||
            localGrid.GetStaticCellState(goal) != CellState::Free)
        {
            continue;
        }

        const Vector2 goalPosition = localGrid.CellToWorld(goal);
        if (!localGrid.IsStaticFootprintFree(
            goalPosition.x,
            goalPosition.y,
            config.robotRadius))
        {
            continue;
        }

        if (std::none_of(
            goals.begin(),
            goals.end(),
            [&goal](const GridCell& existing)
            {
                return SameCell(existing, goal);
            }))
        {
            goals.push_back(goal);
        }
    }

    if (goals.empty())
        return Fail(FailureReason::GoalUnreachable);

    const auto& trajectories =
        localGrid.GetCachedObstacleTrajectories();

    // Vérification du cache des obstacles dynamiques.
    if (localGrid.HasRelevantDynamicObstacles())
    {
        if (trajectories.empty())
            return Fail(FailureReason::MissingTrajectoryCache);

        for (const std::uint32_t id :
            localGrid.GetRelevantObstacleIds())
        {
            const auto it = std::find_if(
                trajectories.begin(),
                trajectories.end(),
                [id](const CachedObstacleTrajectory& trajectory)
                {
                    return trajectory.obstacleId == id;
                }
            );

            if (it == trajectories.end())
                return Fail(FailureReason::MissingRelevantTrajectory);
        }
    }

    if (std::any_of(
        goals.begin(),
        goals.end(),
        [&start](const GridCell& goal)
        {
            return SameCell(start, goal);
        }))
    {
        outputPath.push_back({ start, 0.0f });
        lastSearchSucceeded = true;
        return true;
    }

    const int maxTimeLayer = static_cast<int>(
        std::floor(
            config.predictionHorizon / config.timeStep
        )
    );

    if (maxTimeLayer <= 0)
        return Fail(FailureReason::InvalidTimeHorizon);

    const std::size_t cellsPerLayer =
        static_cast<std::size_t>(width) * height;

    const std::size_t totalStates =
        cellsPerLayer *
        static_cast<std::size_t>(maxTimeLayer + 1);

    const float infinity =
        std::numeric_limits<float>::infinity();

    std::vector<float> gCost(totalStates, infinity);

    std::vector<std::size_t> parent(
        totalStates,
        std::numeric_limits<std::size_t>::max()
    );

    std::vector<std::uint8_t> closed(totalStates, 0);

    auto GetIndex = [&](int x, int z, int timeLayer)
    {
        return
            static_cast<std::size_t>(timeLayer) *
            cellsPerLayer +
            static_cast<std::size_t>(z) * width +
            x;
    };

    auto DecodeIndex = [&](std::size_t index,
        GridCell& cell,
        int& timeLayer)
    {
        timeLayer = static_cast<int>(
            index / cellsPerLayer
        );

        const std::size_t localIndex =
            index % cellsPerLayer;

        cell.x = static_cast<int>(localIndex % width);
        cell.z = static_cast<int>(localIndex / width);
    };

    std::priority_queue<
        SearchNode,
        std::vector<SearchNode>,
        CompareNodes
    > openSet;

    const std::size_t startIndex =
        GetIndex(start.x, start.z, 0);

    gCost[startIndex] = 0.0f;

    openSet.push({
        startIndex,
        0.0f,
        Heuristic(
            start,
            goals,
            localGrid.GetCellSize(),
            config.planningSpeed,
            config.diagonalCost
        )
    });

    // Huit directions + attente.
    constexpr std::array<GridCell, 9> actions =
    {{
        {  1,  0 },
        { -1,  0 },
        {  0,  1 },
        {  0, -1 },
        {  1,  1 },
        {  1, -1 },
        { -1,  1 },
        { -1, -1 },
        {  0,  0 }
    }};

    int expandedNodes = 0;

    std::size_t goalIndex =
        std::numeric_limits<std::size_t>::max();
    bool touchedLocalBoundary = false;

    while (!openSet.empty())
    {
        const SearchNode current = openSet.top();
        openSet.pop();

        if (current.g > gCost[current.index] + EPSILON)
            continue;

        if (closed[current.index])
            continue;

        if (expandedNodes >= config.maxExpandedNodes)
            return Fail(FailureReason::SearchLimitReached);

        closed[current.index] = 1;

        ++expandedNodes;
        lastExpandedNodes = expandedNodes;
        lastDiagnostics.expandedNodes = expandedNodes;

        GridCell currentCell;
        int currentLayer = 0;

        DecodeIndex(
            current.index,
            currentCell,
            currentLayer
        );
        lastDiagnostics.deepestTimeLayer = std::max(
            lastDiagnostics.deepestTimeLayer,
            currentLayer
        );
        touchedLocalBoundary =
            touchedLocalBoundary ||
            currentCell.x == 0 ||
            currentCell.z == 0 ||
            currentCell.x == width - 1 ||
            currentCell.z == height - 1;

        if (std::any_of(
            goals.begin(),
            goals.end(),
            [&currentCell](const GridCell& goal)
            {
                return SameCell(currentCell, goal);
            }))
        {
            goalIndex = current.index;
            break;
        }

        const Vector2 currentPosition =
            localGrid.CellToWorld(currentCell);
        bool hasValidSuccessor = false;

        for (const GridCell& action : actions)
        {
            ++lastDiagnostics.rejectedNeighbors;

            const bool waiting =
                action.x == 0 && action.z == 0;

            GridCell nextCell =
            {
                currentCell.x + action.x,
                currentCell.z + action.z
            };

            if (nextCell.x < 0 || nextCell.z < 0 ||
                nextCell.x >= width || nextCell.z >= height)
            {
                ++lastDiagnostics.rejectedLocalWindow;
                continue;
            }

            if (!waiting &&
                localGrid.GetStaticCellState(nextCell) !=
                CellState::Free)
            {
                ++lastDiagnostics.rejectedCollisions;
                ++lastDiagnostics.rejectedStaticObstacles;
                continue;
            }

            const float cellSize =
                localGrid.GetCellSize();

            const float moveDistance =
                waiting
                ? 0.0f
                : cellSize * (
                    action.x != 0 && action.z != 0
                    ? config.diagonalCost
                    : 1.0f
                );

            const float moveDuration =
                waiting
                ? config.timeStep
                : moveDistance / config.planningSpeed;

            const int durationLayers = std::max(
                1,
                static_cast<int>(std::ceil(
                    moveDuration / config.timeStep
                ))
            );

            const int nextLayer =
                currentLayer + durationLayers;

            if (nextLayer > maxTimeLayer)
            {
                ++lastDiagnostics.rejectedTimeHorizon;
                continue;
            }

            const float actualDuration =
                durationLayers * config.timeStep;

            const Vector2 nextPosition =
                localGrid.CellToWorld(nextCell);

            const SegmentSafety segmentSafety = IsSegmentSafe(
                localGrid,
                trajectories,
                currentPosition,
                nextPosition,
                currentLayer * config.timeStep,
                actualDuration,
                config.timeStep,
                config.robotRadius);
            if (segmentSafety != SegmentSafety::Safe)
            {
                if (segmentSafety == SegmentSafety::PredictionUnavailable)
                    ++lastDiagnostics.rejectedTimeHorizon;
                else if (segmentSafety ==
                    SegmentSafety::LocalWindowExceeded)
                {
                    ++lastDiagnostics.rejectedLocalWindow;
                }
                else
                    ++lastDiagnostics.rejectedCollisions;
                if (segmentSafety == SegmentSafety::StaticCollision)
                    ++lastDiagnostics.rejectedStaticObstacles;
                continue;
            }

            hasValidSuccessor = true;
            const std::size_t nextIndex =
                GetIndex(
                    nextCell.x,
                    nextCell.z,
                    nextLayer
                );

            if (closed[nextIndex])
                continue;

            float traversalCost =
                localGrid.GetTraversalCost(nextCell);

            if (!std::isfinite(traversalCost) ||
                traversalCost <= 0.0f)
            {
                traversalCost = 1.0f;
            }

            const float edgeCost =
                actualDuration *
                (traversalCost + (waiting ? config.waitCost : 0.0f));

            const float tentativeG =
                gCost[current.index] + edgeCost;

            if (tentativeG + EPSILON >= gCost[nextIndex])
                continue;

            gCost[nextIndex] = tentativeG;
            parent[nextIndex] = current.index;

            const float h = Heuristic(
                nextCell,
                goals,
                cellSize,
                config.planningSpeed,
                config.diagonalCost
            );

            openSet.push({
                nextIndex,
                tentativeG,
                tentativeG + h
            });
        }

        if (!hasValidSuccessor)
            ++lastDiagnostics.statesWithoutSuccessor;
    }

    if (goalIndex ==
        std::numeric_limits<std::size_t>::max())
    {
        if (lastDiagnostics.deepestTimeLayer >= maxTimeLayer &&
            lastDiagnostics.rejectedTimeHorizon > 0)
        {
            return Fail(FailureReason::PredictionHorizonExceeded);
        }

        if (touchedLocalBoundary &&
            lastDiagnostics.rejectedLocalWindow > 0)
        {
            return Fail(FailureReason::LocalWindowExceeded);
        }

        if (expandedNodes > 0 &&
            lastDiagnostics.statesWithoutSuccessor == expandedNodes)
        {
            return Fail(FailureReason::NoValidSuccessor);
        }

        return Fail(FailureReason::GoalUnreachable);
    }

    // ========================================
    // RECONSTRUCTION DU CHEMIN
    // ========================================

    std::vector<TemporalWaypoint> reversedPath;

    std::size_t index = goalIndex;

    while (index != startIndex)
    {
        GridCell cell;
        int timeLayer = 0;

        DecodeIndex(index, cell, timeLayer);

        reversedPath.push_back({
            cell,
            timeLayer * config.timeStep
        });

        index = parent[index];

        if (index == std::numeric_limits<std::size_t>::max())
        {
            outputPath.clear();
            return Fail(FailureReason::ReconstructionFailed);
        }
    }

    reversedPath.push_back({ start, 0.0f });

    outputPath.assign(
        reversedPath.rbegin(),
        reversedPath.rend()
    );

    lastSearchSucceeded = true;
    lastFailureReason = FailureReason::None;

    return true;
}
