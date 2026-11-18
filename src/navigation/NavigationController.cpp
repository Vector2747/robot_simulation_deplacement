#include "NavigationController.h"
#include "../physics/CollisionChecker.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>


namespace
{
    bool IsClearOfStaticObstacles(
        const World& world,
        const Vector3& position,
        float radius
    )
    {
        std::vector<Obstacle> staticObstacles;
        for (const Obstacle& obstacle : world.GetObstacles())
        {
            if (!obstacle.isMoving)
                staticObstacles.push_back(obstacle);
        }

        return CollisionChecker::IsPositionSafe(
            position,
            radius,
            world.GetTerrain(),
            staticObstacles
        );
    }

    bool FindNearbyTraversableCell(
        const GridMap& grid,
        const GridCell& origin,
        GridCell& result
    )
    {
        constexpr int maximumSearchRadius = 4;

        for (int radius = 1; radius <= maximumSearchRadius; ++radius)
        {
            float nearestDistanceSquared =
                std::numeric_limits<float>::infinity();
            bool found = false;

            for (int z = origin.z - radius;
                 z <= origin.z + radius;
                 ++z)
            {
                for (int x = origin.x - radius;
                     x <= origin.x + radius;
                     ++x)
                {
                    if (std::max(
                        std::abs(x - origin.x),
                        std::abs(z - origin.z)) != radius)
                    {
                        continue;
                    }

                    const GridCell candidate = { x, z };
                    if (!grid.IsTraversable(candidate))
                        continue;

                    const float dx =
                        static_cast<float>(x - origin.x);
                    const float dz =
                        static_cast<float>(z - origin.z);
                    const float distanceSquared =
                        dx * dx + dz * dz;

                    if (distanceSquared < nearestDistanceSquared)
                    {
                        nearestDistanceSquared = distanceSquared;
                        result = candidate;
                        found = true;
                    }
                }
            }

            if (found)
                return true;
        }

        return false;
    }

    float DistanceToObstacle(
        float x,
        float z,
        const PredictedObstaclePoint& obstacle
    )
    {
        const float halfWidth = std::max(0.0f, obstacle.size.x * 0.5f);
        const float halfDepth = std::max(0.0f, obstacle.size.z * 0.5f);

        const float closestX = std::clamp(
            x,
            obstacle.position.x - halfWidth,
            obstacle.position.x + halfWidth
        );

        const float closestZ = std::clamp(
            z,
            obstacle.position.z - halfDepth,
            obstacle.position.z + halfDepth
        );

        const float dx = x - closestX;
        const float dz = z - closestZ;
        return std::sqrt(dx * dx + dz * dz);
    }

    bool GetCachedObstacleAtTime(
        const CachedObstacleTrajectory& trajectory,
        float time,
        PredictedObstaclePoint& result
    )
    {
        const auto& samples = trajectory.samples;

        if (samples.empty() ||
            time < samples.front().time ||
            time > samples.back().time)
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

        const auto& a = *(upper - 1);
        const auto& b = *upper;

        const float duration = b.time - a.time;

        const float alpha = duration > 0.0001f
            ? (time - a.time) / duration
            : 0.0f;

        result = a;

        result.position.x =
            a.position.x + (b.position.x - a.position.x) * alpha;

        result.position.z =
            a.position.z + (b.position.z - a.position.z) * alpha;

        result.size.x =
            a.size.x + (b.size.x - a.size.x) * alpha;

        result.size.z =
            a.size.z + (b.size.z - a.size.z) * alpha;

        result.time = time;

        return true;
    }
}


// ============================================
// INITIALISATION DE LA NAVIGATION
// ============================================

bool NavigationController::Initialize(
    const World& world,
    const Robot& robot,
    const Target& target
)
{
    // Création de la grille globale.
    globalGrid = std::make_unique<GridMap>(
        world.GetTerrain()
    );

    // Synchronisation initiale :
    // insertion des obstacles statiques dans la grille.
    gridSynchronizer.Synchronize(
        world,
        *globalGrid,
        robot.GetRadius()
    );

    // Initialisation de la grille locale.
    // 40 × 40 cellules avec une résolution de 1 mètre.
    localGrid.Initialize(
        40,
        40,
        1.0f
    );

    // Positionnement initial autour du robot.
    const Vector3 initialRobotPosition = robot.GetPosition();

    localGrid.UpdateWindow(
        initialRobotPosition.x,
        initialRobotPosition.z
    );

    // Synchronisation initiale avec la grille globale.
    localGrid.InvalidateAllCells();

    localGrid.UpdateFromGlobalGrid(*globalGrid);

    ConfigureMotionLimits(robot);

    temporalPath.clear();
    temporalPathValid = false;
    temporalPlanningRequested = false;
    previousTemporalThreat = false;
    lastTemporalTriggerThreatCount = 0;
    lastTemporalTriggerTtc = -1.0f;
    pendingTemporalTriggerReason.clear();
    temporalDiagnostics = {};
    noPathThreatGraceRemaining = 0.0f;

    // Conversion des positions en cellules.
    GridCell startCell;
    GridCell goalCell;

    Vector3 robotPosition = robot.GetPosition();
    Vector3 targetPosition = target.GetPosition();

    if (!globalGrid->WorldToCell(
        robotPosition.x,
        robotPosition.z,
        startCell
    ))
    {
        return false;
    }

    if (!globalGrid->WorldToCell(
        targetPosition.x,
        targetPosition.z,
        goalCell
    ))
    {
        return false;
    }

    // La cellule de départ peut être bloquée par la marge conservative
    // même si le robot reste physiquement séparé des obstacles.
    if (!globalGrid->IsTraversable(startCell) &&
        IsClearOfStaticObstacles(
            world,
            robotPosition,
            robot.GetRadius()))
    {
        GridCell nearbyCell;
        if (FindNearbyTraversableCell(
            *globalGrid,
            startCell,
            nearbyCell))
        {
            startCell = nearbyCell;
        }
    }

    if (!globalGrid->IsTraversable(startCell) ||
        !globalGrid->IsTraversable(goalCell))
    {
        return false;
    }

    // Initialisation de D* Lite.
    if (!globalPlanner.Initialize(
        *globalGrid,
        startCell,
        goalCell
    ))
    {
        return false;
    }

    hasStartCell = true;
    hasGoalCell = true;

    lastStartCell = startCell;
    lastGoalCell = goalCell;

    // Premier calcul du chemin.
    const bool pathComputed = globalPlanner.ComputePath();

    globalPath = globalPlanner.GetPath();

    if (!pathComputed || globalPath.empty())
    {
        TraceLog(
            LOG_WARNING,
            "D* Lite: no valid global path found."
        );

        return false;
    }

    TraceLog(
        LOG_INFO,
        TextFormat(
            "D* Lite: global path computed (%i cells).",
            static_cast<int>(globalPath.size())
        )
    );

    // Les modifications initiales ont été prises en compte.
    globalGrid->ClearChangedCells();

    return true;
}


// ============================================
// MISE À JOUR DE LA NAVIGATION
// ============================================

void NavigationController::Update(
    const World& world,
    const Robot& robot,
    const Target& target,
    const NavigationConfig& config,
    float dt
)
{
    if (!globalGrid)
    {
        state = NavigationState::Idle;
        globalPath.clear();
        return;
    }

    if (temporalPathValid)
    {
        temporalPathElapsed += std::max(0.0f, dt);
    }

    bool mustRecomputePath = false;
    bool mustReinitializePlanner = false;
    bool invalidatedTemporalPath = false;
    auto clearTemporalPath = [&]()
    {
        invalidatedTemporalPath =
            invalidatedTemporalPath ||
            temporalPathValid ||
            !temporalPath.empty();
        temporalPath.clear();
        temporalPathValid = false;
        temporalPathElapsed = 0.0f;
        temporalPathIndex = 0;
    };

    // ----------------------------------------
    // 1. Synchronisation monde / grille globale
    // ----------------------------------------

    GridSyncResult syncResult =
        gridSynchronizer.Synchronize(
            world,
            *globalGrid,
            robot.GetRadius()
        );

    // ----------------------------------------
    // Synchronisation de la grille locale
    // ----------------------------------------

    // Position actuelle du robot.
    const Vector3 localRobotPosition = robot.GetPosition();

    // Déplacer la fenêtre locale autour du robot.
    const bool localWindowMoved = localGrid.UpdateWindow(
        localRobotPosition.x,
        localRobotPosition.z
    );

    // Si la fenêtre a changé de position,
    // les nouvelles cellules doivent être synchronisées.
    if (localWindowMoved)
    {
        clearTemporalPath();
        localGrid.UpdateFromGlobalGrid(*globalGrid);
    }

    // ----------------------------------------
    // Mise à jour prédictive des obstacles mobiles
    // ----------------------------------------

    constexpr float predictionHorizon = 3.0f;//0.5f;

    // Actualisation limitée à 25 Hz.
    // Le rayon du robot sert de marge de sécurité.
    const bool dynamicCacheUpdated =
        localGrid.UpdateDynamicObstacles(
            world,
            dt,
            predictionHorizon,
            robot.GetRadius()
        );

    temporalPlanningAccumulator += std::max(0.0f, dt);
    noPathThreatGraceRemaining = std::max(
        0.0f,
        noPathThreatGraceRemaining - std::max(0.0f, dt)
    );

    auto publishThreatDiagnostics = [&](
        const TemporalThreatAssessment& assessment)
    {
        temporalDiagnostics.threatsDetected =
            assessment.threatsDetected;
        temporalDiagnostics.rearThreatsDetected =
            assessment.rearThreatsDetected;
        temporalDiagnostics.minimumTimeToCollision =
            assessment.minimumTimeToCollision;
        temporalDiagnostics.mostDangerousObstacleId =
            assessment.mostDangerousObstacleId;
    };

    auto holdWithoutGlobalPath = [&]()
    {
        globalPath.clear();
        clearTemporalPath();
        temporalPlanningRequested = false;
        previousTemporalThreat = false;
        lastTemporalTriggerThreatCount = 0;
        lastTemporalTriggerTtc = -1.0f;
        pendingTemporalTriggerReason.clear();

        if (dynamicCacheUpdated)
        {
            const TemporalThreatAssessment assessment =
                AssessTemporalThreats(robot, 3.0f);
            publishThreatDiagnostics(assessment);
            if (assessment.IsThreatened())
                noPathThreatGraceRemaining = 0.35f;
        }

        globalGrid->ClearChangedCells();
        state = NavigationState::Idle;
    };

    if (syncResult == GridSyncResult::FullRebuild)
    {
        clearTemporalPath();

        mustReinitializePlanner = true;

        // Le Global Grid a été entièrement reconstruit.
        // Il faut resynchroniser toute la fenêtre locale.
        localGrid.InvalidateAllCells();

        localGrid.UpdateFromGlobalGrid(*globalGrid);
    }
    else if (syncResult == GridSyncResult::IncrementalUpdate)
    {
        clearTemporalPath();

        // Transmettre les modifications au LocalGrid.
        localGrid.UpdateFromGlobalChanges(
            *globalGrid,
            globalGrid->GetChangedCells()
        );

        // Informer D* Lite de chaque cellule modifiée.
        for (const GridCellChange& change :
             globalGrid->GetChangedCells())
        {
            globalPlanner.NotifyCellChanged(change.cell);
        }

        mustRecomputePath = true;
    }

    if (invalidatedTemporalPath)
    {
        temporalPlanningRequested = true;
        pendingTemporalTriggerReason = "Detour temporel invalide";
    }

    // ----------------------------------------
    // 2. Conversion des positions
    // ----------------------------------------

    GridCell currentStartCell;
    GridCell currentGoalCell;

    Vector3 robotPosition = robot.GetPosition();
    Vector3 targetPosition = target.GetPosition();

    bool validStart = globalGrid->WorldToCell(
        robotPosition.x,
        robotPosition.z,
        currentStartCell
    );

    bool validGoal = globalGrid->WorldToCell(
        targetPosition.x,
        targetPosition.z,
        currentGoalCell
    );

    const bool goalCellChanged =
        validGoal &&
        (!hasGoalCell ||
         currentGoalCell.x != lastGoalCell.x ||
         currentGoalCell.z != lastGoalCell.z);
    if (goalCellChanged)
        clearTemporalPath();

    // Impossible de planifier hors de la grille.
    if (!validStart || !validGoal)
    {
        if (!validStart)
            hasStartCell = false;
        if (!validGoal)
        {
            hasGoalCell = false;
        }
        holdWithoutGlobalPath();
        return;
    }

    // La marge de sécurité de la grille globale est plus conservative
    // que la collision géométrique exacte. Ne pas perdre le chemin si le
    // robot est dans une cellule tampon mais reste physiquement dégagé.
    if (!globalGrid->IsTraversable(currentStartCell) &&
        IsClearOfStaticObstacles(
            world,
            robotPosition,
            robot.GetRadius()))
    {
        GridCell nearbyCell;
        if (FindNearbyTraversableCell(
            *globalGrid,
            currentStartCell,
            nearbyCell))
        {
            currentStartCell = nearbyCell;
        }
    }

    if (!globalGrid->IsTraversable(currentStartCell) ||
        !globalGrid->IsTraversable(currentGoalCell))
    {
        if (!globalGrid->IsTraversable(currentGoalCell))
            hasGoalCell = false;
        holdWithoutGlobalPath();
        return;
    }

    // ----------------------------------------
    // 3. Réinitialisation si nécessaire
    // ----------------------------------------

    if (mustReinitializePlanner ||
        !globalPlanner.IsInitialized())
    {
        if (!globalPlanner.Initialize(
            *globalGrid,
            currentStartCell,
            currentGoalCell
        ))
        {
            holdWithoutGlobalPath();
            return;
        }

        lastStartCell = currentStartCell;
        lastGoalCell = currentGoalCell;

        hasStartCell = true;
        hasGoalCell = true;

        mustRecomputePath = true;
    }
    else
    {
        // ------------------------------------
        // 4. Mise à jour de la position du robot
        // ------------------------------------

        if (!hasStartCell ||
            currentStartCell.x != lastStartCell.x ||
            currentStartCell.z != lastStartCell.z)
        {
            const bool startNeedsInitialization = !hasStartCell;
            globalPlanner.UpdateStart(currentStartCell);

            lastStartCell = currentStartCell;
            hasStartCell = true;

            mustRecomputePath =
                mustRecomputePath ||
                startNeedsInitialization ||
                !globalPath.empty();
        }

        // ------------------------------------
        // 5. Mise à jour de la cible
        // ------------------------------------

        if (!hasGoalCell ||
            currentGoalCell.x != lastGoalCell.x ||
            currentGoalCell.z != lastGoalCell.z)
        {
            if (!globalPlanner.UpdateGoal(currentGoalCell))
            {
                hasGoalCell = false;
                holdWithoutGlobalPath();
                return;
            }

            lastGoalCell = currentGoalCell;
            hasGoalCell = true;

            mustRecomputePath = true;
        }
    }

    // ----------------------------------------
    // 6. Recalcul du chemin si nécessaire
    // ----------------------------------------

    if (mustRecomputePath)
    {
        globalPlanner.ComputePath();

        globalPath = globalPlanner.GetPath();
    }

    if (globalPath.empty())
    {
        holdWithoutGlobalPath();
        return;
    }


    // ----------------------------------------
    // 6.5 bis. Planification temporelle locale
    // ----------------------------------------

    const bool relevantObstacleSetChanged =
        localGrid.ConsumeRelevantObstacleSetChanged();

    if (dynamicCacheUpdated &&
        temporalPathValid &&
        !IsTemporalPathSafe(robot))
    {
        temporalPath.clear();
        temporalPathValid = false;
        temporalPathElapsed = 0.0f;
        temporalPlanningRequested = true;
        invalidatedTemporalPath = true;
    }

    TemporalThreatAssessment threatAssessment;
    if (dynamicCacheUpdated)
    {
        threatAssessment = AssessTemporalThreats(robot);

        publishThreatDiagnostics(threatAssessment);

        const bool threatIncreased =
            threatAssessment.IsThreatened() &&
            (!previousTemporalThreat ||
             threatAssessment.threatsDetected >
                 lastTemporalTriggerThreatCount ||
             (lastTemporalTriggerTtc >= 0.0f &&
              threatAssessment.minimumTimeToCollision >= 0.0f &&
              threatAssessment.minimumTimeToCollision <
                  lastTemporalTriggerTtc - 0.25f));

        if (threatAssessment.IsThreatened() &&
            (threatIncreased ||
             relevantObstacleSetChanged ||
             invalidatedTemporalPath))
        {
            temporalPlanningRequested = true;
            lastTemporalTriggerThreatCount =
                threatAssessment.threatsDetected;
            lastTemporalTriggerTtc =
                threatAssessment.minimumTimeToCollision;
            pendingTemporalTriggerReason =
                invalidatedTemporalPath
                ? "Detour temporel invalide"
                : threatAssessment.rearThreatsDetected > 0
                    ? "TTC: obstacle venant de l'arriere"
                    : threatAssessment.robotThreatened
                        ? "TTC: menace directe sur le robot"
                        : threatAssessment.threatsDetected > 1
                            ? "Plusieurs obstacles convergent sur la trajectoire"
                            : "Collision predite sur le chemin global";
        }

        if (!threatAssessment.IsThreatened())
        {
            if (temporalPathValid)
            {
                temporalPath.clear();
                temporalPathValid = false;
                temporalPathElapsed = 0.0f;
            }

            temporalPlanningRequested = false;
            pendingTemporalTriggerReason.clear();
        }

        previousTemporalThreat = threatAssessment.IsThreatened();
    }

    const bool urgentTemporalThreat =
        threatAssessment.IsThreatened() &&
        threatAssessment.minimumTimeToCollision >= 0.0f &&
        threatAssessment.minimumTimeToCollision <= 1.0f;
    const bool firstTemporalSearch =
        temporalDiagnostics.searchesRun == 0;

    if (dynamicCacheUpdated &&
        !temporalPathValid &&
        temporalPlanningRequested &&
        (temporalPlanningAccumulator >= temporalPlanningInterval ||
         relevantObstacleSetChanged ||
         urgentTemporalThreat ||
         firstTemporalSearch))
    {
        temporalPlanningAccumulator = 0.0f;

        if (pendingTemporalTriggerReason.empty())
            pendingTemporalTriggerReason = "Danger temporel confirme";
        temporalDiagnostics.lastTriggerReason =
            pendingTemporalTriggerReason;

        GridCell localStart;
        std::vector<GridCell> localGoals;
        const Vector3 robotPosition = robot.GetPosition();

        const bool validLocalStart =
            localGrid.WorldToCell(
                robotPosition.x,
                robotPosition.z,
                localStart
            );
        const bool validLocalGoals =
            !globalPath.empty() &&
            FindTemporalGoals(robot, localGoals);

        if (validLocalStart && validLocalGoals)
        {
            temporalPath.clear();
            const auto searchStart =
                std::chrono::steady_clock::now();

            temporalPathValid = temporalPlanner.FindPath(
                localGrid,
                localStart,
                localGoals,
                temporalPath
            );

            const auto searchEnd =
                std::chrono::steady_clock::now();
            temporalDiagnostics.lastSearchDurationMilliseconds =
                std::chrono::duration<float, std::milli>(
                    searchEnd - searchStart
                ).count();
            temporalDiagnostics.recentSearchDurationsMilliseconds.push_back(
                temporalDiagnostics.lastSearchDurationMilliseconds
            );
            if (temporalDiagnostics.recentSearchDurationsMilliseconds.size() >
                32)
            {
                temporalDiagnostics.recentSearchDurationsMilliseconds.erase(
                    temporalDiagnostics.recentSearchDurationsMilliseconds.begin()
                );
            }

            ++temporalDiagnostics.searchesRun;
            temporalDiagnostics.lastExpandedNodes =
                temporalPlanner.GetLastExpandedNodes();
            const TemporalAStarSearchDiagnostics& searchDiagnostics =
                temporalPlanner.GetLastDiagnostics();
            temporalDiagnostics.lastRejectedNeighbors =
                searchDiagnostics.rejectedNeighbors;
            temporalDiagnostics.lastRejectedCollisions =
                searchDiagnostics.rejectedCollisions;
            temporalDiagnostics.lastRejectedTimeHorizon =
                searchDiagnostics.rejectedTimeHorizon;
            temporalDiagnostics.lastRejectedLocalWindow =
                searchDiagnostics.rejectedLocalWindow;
            temporalDiagnostics.totalExpandedNodes +=
                static_cast<std::uint64_t>(
                    temporalDiagnostics.lastExpandedNodes
                );
            temporalDiagnostics.lastSearchFailureReason =
                temporalPlanner.GetLastFailureReason();

            if (temporalPathValid)
            {
                ++temporalDiagnostics.searchesSucceeded;
                temporalPathElapsed = 0.0f;
                temporalPathIndex = 0;
            }
            else
            {
                ++temporalDiagnostics.searchesFailed;
                temporalPath.clear();
            }
        }
        else
        {
            temporalPath.clear();
            temporalPathValid = false;
            temporalDiagnostics.lastSearchFailureReason =
                validLocalStart
                ? "Aucun objectif temporel libre apres la menace"
                : "StartBlocked: depart hors LocalGrid";
        }

        temporalPlanningRequested = false;
        pendingTemporalTriggerReason.clear();
    }

    // Les changements de grille ont été transmis.
    globalGrid->ClearChangedCells();

    // ----------------------------------------
    // 7. État de navigation
    // ----------------------------------------

    float distance = GetDistanceToTarget(
        robot,
        target
    );

    if (!std::isfinite(distance) ||
        !std::isfinite(config.arrivalRadius))
    {
        state = NavigationState::Idle;
        return;
    }

    float arrivalRadius = std::max(
        0.0f,
        config.arrivalRadius
    );

    if (distance <= arrivalRadius)
    {
        state = NavigationState::TargetReached;
        return;
    }

    Vector2 velocity = robot.GetVelocity();

    float speedSquared =
        velocity.x * velocity.x +
        velocity.y * velocity.y;

    state = (speedSquared > 0.0001f)
        ? NavigationState::Moving
        : NavigationState::Idle;
}


// ============================================
// DISTANCE ROBOT / CIBLE
// ============================================

float NavigationController::GetDistanceToTarget(
    const Robot& robot,
    const Target& target
) const
{
    Vector3 robotPosition = robot.GetPosition();
    Vector3 targetPosition = target.GetPosition();

    float dx = targetPosition.x - robotPosition.x;
    float dz = targetPosition.z - robotPosition.z;

    return std::sqrt(dx * dx + dz * dz);
}


// ============================================
// ACCESSEURS
// ============================================

NavigationState NavigationController::GetState() const
{
    return state;
}


const std::vector<GridCell>&
NavigationController::GetGlobalPath() const
{
    return globalPath;
}


const GridMap* NavigationController::GetGlobalGrid() const
{
    return globalGrid.get();
}

// ============================================
// CALCUL DE LA VITESSE AUTOMATIQUE
// ============================================


Vector2 NavigationController::CalculateDesiredVelocity(
    const Robot& robot,
    const Target& target,
    const float dt,
    const NavigationConfig& config,
    const World& world
)
{
    ConfigureMotionLimits(robot);

    if (!globalGrid)
        return { 0.0f, 0.0f };

    if (globalPath.empty())
    {
        const bool defensiveMotionActive =
            temporalDiagnostics.threatsDetected > 0 ||
            noPathThreatGraceRemaining > 0.0f;
        if (!defensiveMotionActive ||
            !localGrid.IsInitialized())
        {
            localPlanner.ResetMotionState();
            return { 0.0f, 0.0f };
        }

        const Vector3 robotPosition = robot.GetPosition();
        Vector2 escapeDirection = robot.GetVelocity();
        const std::uint32_t dangerousObstacleId =
            temporalDiagnostics.mostDangerousObstacleId;
        const auto& obstacles = world.GetObstacles();
        const auto dangerousObstacle = std::find_if(
            obstacles.begin(),
            obstacles.end(),
            [dangerousObstacleId](const Obstacle& obstacle)
            {
                return dangerousObstacleId != 0 &&
                    obstacle.id == dangerousObstacleId &&
                    obstacle.isMoving;
            }
        );
        if (dangerousObstacle != obstacles.end())
        {
            escapeDirection = {
                robotPosition.x - dangerousObstacle->position.x,
                robotPosition.z - dangerousObstacle->position.z
            };
        }
        const float escapeLength = std::sqrt(
            escapeDirection.x * escapeDirection.x +
            escapeDirection.y * escapeDirection.y
        );
        if (escapeLength > 0.0001f)
        {
            escapeDirection.x =
                escapeDirection.x / escapeLength * robot.GetMaxSpeed();
            escapeDirection.y =
                escapeDirection.y / escapeLength * robot.GetMaxSpeed();
        }

        return localPlanner.CalculateVelocity(
            robotPosition,
            escapeDirection,
            robotPosition,
            globalPath,
            *globalGrid,
            dt,
            localGrid,
            world,
            temporalDiagnostics.threatsDetected > 0,
            true
        );
    }

    if (state == NavigationState::TargetReached)
    {
        return { 0.0f, 0.0f };
    }

    Vector2 desiredVelocity = { 0.0f, 0.0f };
    bool temporalCommandFound = false;

    // ----------------------------------------
    // 1. Suivi du chemin temporel
    // ----------------------------------------

    if (!temporalCommandFound &&
        temporalPathValid &&
        !temporalPath.empty())
    {
        const Vector3 robotPosition = robot.GetPosition();

        constexpr float waypointTolerance = 0.20f;
        constexpr float scheduleLatenessTolerance = 0.30f;

        for (std::size_t i = 1; i < temporalPath.size(); ++i)
        {
            const TemporalWaypoint& waypoint = temporalPath[i];
            const TemporalWaypoint& previous = temporalPath[i - 1];

            const float remainingTime =
                waypoint.arrivalTime - temporalPathElapsed;

            // Une même cellule consécutive représente une attente.
            const bool isWaiting =
                waypoint.cell.x == previous.cell.x &&
                waypoint.cell.z == previous.cell.z;

            // Attente temporelle explicite.
            if (isWaiting)
            {
                if (remainingTime > 0.05f)
                {
                    desiredVelocity = { 0.0f, 0.0f };
                    temporalCommandFound = true;
                    break;
                }

                continue;
            }

            const Vector2 waypointPosition =
                localGrid.CellToWorld(waypoint.cell);

            const float dx =
                waypointPosition.x - robotPosition.x;

            const float dz =
                waypointPosition.y - robotPosition.z;

            const float distance =
                std::sqrt(dx * dx + dz * dz);

            // Le robot est arrivé trop tôt :
            // il attend l'instant prévu avant de continuer.
            if (distance <= waypointTolerance)
            {
                if (remainingTime > 0.05f)
                {
                    desiredVelocity = { 0.0f, 0.0f };
                    temporalCommandFound = true;
                    break;
                }

                continue;
            }

            // Le robot est trop en retard :
            // le calendrier de prédiction n'est plus fiable.
            if (remainingTime < -scheduleLatenessTolerance)
            {
                temporalPath.clear();
                temporalPathValid = false;
                temporalPathElapsed = 0.0f;
                break;
            }

            const float inverseDistance = 1.0f / distance;

            const Vector2 direction = {
                dx * inverseDistance,
                dz * inverseDistance
            };

            const float timeToUse =
                std::max(0.10f, remainingTime);

            const float requiredSpeed =
                distance / timeToUse;

            const bool urgentRearThreat =
                temporalDiagnostics.rearThreatsDetected > 0 &&
                temporalDiagnostics.minimumTimeToCollision >= 0.0f &&
                temporalDiagnostics.minimumTimeToCollision <= 1.0f;
            const float speed = urgentRearThreat
                ? robot.GetMaxSpeed()
                : std::min(
                    robot.GetMaxSpeed(),
                    requiredSpeed
                );

            desiredVelocity = {
                direction.x * speed,
                direction.y * speed
            };

            temporalCommandFound = true;
            break;
        }

        // Le détour est terminé.
        if (!temporalCommandFound &&
            temporalPathValid &&
            !temporalPath.empty())
        {
            temporalPath.clear();
            temporalPathValid = false;
            temporalPathElapsed = 0.0f;
        }
    }


    // ----------------------------------------
    // 2. Suivi du chemin global D* Lite
    // ----------------------------------------

    if (!temporalCommandFound)
    {
        desiredVelocity =
            pathFollower.CalculateDesiredVelocity(
                robot,
                target,
                *globalGrid,
                globalPath,
                config.arrivalRadius
            );
    }

    // ----------------------------------------
    // 3. Contrôle final de sécurité
    // ----------------------------------------

    return localPlanner.CalculateVelocity(
        robot.GetPosition(),
        desiredVelocity,
        target.GetPosition(),
        globalPath,
        *globalGrid,
        dt,
        localGrid,
        world,
        temporalDiagnostics.threatsDetected > 0
    );
}


void NavigationController::ConfigureMotionLimits(const Robot& robot)
{
    const float maxSpeed = robot.GetMaxSpeed();
    const float radius = robot.GetRadius();

    localPlanner.Configure(maxSpeed, radius);

    TemporalAStarConfig temporalConfig;
    temporalConfig.planningSpeed = std::max(
        0.5f,
        maxSpeed * 0.85f
    );
    temporalConfig.robotRadius = radius;
    temporalPlanner.Configure(temporalConfig);
}


const LocalGrid* NavigationController::GetLocalGrid() const
{
    return &localGrid;
}


bool NavigationController::FindTemporalGoals(
    const Robot& robot,
    std::vector<GridCell>& outputGoals
) const
{
    outputGoals.clear();

    if (!globalGrid ||
        globalPath.size() < 2 ||
        !localGrid.IsInitialized())
    {
        return false;
    }

    GridCell robotCell;

    const Vector3 position = robot.GetPosition();

    if (!localGrid.WorldToCell(
        position.x,
        position.z,
        robotCell))
    {
        return false;
    }

    std::size_t nearestPathIndex = 0;
    float nearestDistanceSquared =
        std::numeric_limits<float>::infinity();

    for (std::size_t i = 0; i < globalPath.size(); ++i)
    {
        const Vector2 worldPosition =
            globalGrid->CellToWorld(globalPath[i]);
        const float dx = worldPosition.x - position.x;
        const float dz = worldPosition.y - position.z;
        const float distanceSquared = dx * dx + dz * dz;

        if (distanceSquared < nearestDistanceSquared)
        {
            nearestDistanceSquared = distanceSquared;
            nearestPathIndex = i;
        }
    }

    const float planningSpeed = std::max(
        0.5f,
        robot.GetMaxSpeed() * 0.85f
    );
    const float maximumGoalDistance =
        planningSpeed * 3.0f + localGrid.GetCellSize();
    const float cacheAge = localGrid.GetPredictionCacheAge();
    const auto& trajectories =
        localGrid.GetCachedObstacleTrajectories();

    std::size_t lastThreatenedPathIndex = nearestPathIndex;
    bool hasThreatenedPathPoint = false;
    float threatRouteDistance = 0.0f;
    Vector2 threatPreviousPosition = {
        position.x,
        position.z
    };

    for (std::size_t i = nearestPathIndex;
         i < globalPath.size() &&
             threatRouteDistance <= maximumGoalDistance;
         ++i)
    {
        const Vector2 endpoint =
            globalGrid->CellToWorld(globalPath[i]);
        const float dx = endpoint.x - threatPreviousPosition.x;
        const float dz = endpoint.y - threatPreviousPosition.y;
        const float segmentLength = std::sqrt(dx * dx + dz * dz);

        if (threatRouteDistance + segmentLength >
            maximumGoalDistance)
        {
            break;
        }

        for (const CachedObstacleTrajectory& trajectory :
             trajectories)
        {
            PredictedObstaclePoint obstacle;
            if (GetCachedObstacleAtTime(
                trajectory,
                cacheAge,
                obstacle) &&
                DistanceToObstacle(
                    endpoint.x,
                    endpoint.y,
                    obstacle) <= robot.GetRadius() + 0.15f)
            {
                lastThreatenedPathIndex = i;
                hasThreatenedPathPoint = true;
                break;
            }
        }

        threatRouteDistance += segmentLength;
        threatPreviousPosition = endpoint;
    }

    float routeDistance = 0.0f;
    Vector2 previousPosition = { position.x, position.z };
    for (std::size_t i = nearestPathIndex;
         i < globalPath.size(); ++i)
    {
        const Vector2 worldPosition =
            globalGrid->CellToWorld(globalPath[i]);
        const float dx = worldPosition.x - previousPosition.x;
        const float dz = worldPosition.y - previousPosition.y;
        routeDistance += std::sqrt(dx * dx + dz * dz);

        if (routeDistance > maximumGoalDistance)
            break;

        previousPosition = worldPosition;

        if (hasThreatenedPathPoint &&
            i <= lastThreatenedPathIndex)
        {
            continue;
        }

        GridCell localCell;

        if (!localGrid.WorldToCell(
            worldPosition.x,
            worldPosition.y,
            localCell))
        {
            continue;
        }

        if (localGrid.GetStaticCellState(localCell)
            != CellState::Free)
        {
            continue;
        }

        if (!localGrid.IsStaticFootprintFree(
            worldPosition.x,
            worldPosition.y,
            robot.GetRadius()))
        {
            continue;
        }

        // Éviter de choisir la cellule du robot
        // comme point de reconnexion.
        if (localCell.x == robotCell.x &&
            localCell.z == robotCell.z)
        {
            continue;
        }

        if (std::none_of(
            outputGoals.begin(),
            outputGoals.end(),
            [&localCell](const GridCell& existing)
            {
                return existing.x == localCell.x &&
                    existing.z == localCell.z;
            }))
        {
            outputGoals.push_back(localCell);
        }
    }

    return !outputGoals.empty();
}


NavigationController::TemporalThreatAssessment
NavigationController::AssessTemporalThreats(
    const Robot& robot,
    float reactionHorizon
) const
{
    TemporalThreatAssessment assessment;
    const auto& trajectories =
        localGrid.GetCachedObstacleTrajectories();

    if (!localGrid.HasRelevantDynamicObstacles() ||
        trajectories.empty())
    {
        return assessment;
    }

    const float cacheAge =
        localGrid.GetPredictionCacheAge();
    const float planningSpeed = std::max(
        0.5f,
        robot.GetMaxSpeed() * 0.85f
    );
    constexpr float threatClearance = 0.15f;
    const Vector3 robotPosition = robot.GetPosition();
    const Vector2 robotVelocity = robot.GetVelocity();
    constexpr float routeHorizon = 3.0f;
    constexpr float sampleStep = 0.1f;
    constexpr float epsilon = 0.001f;

    Vector2 heading = robotVelocity;
    float headingLength = std::sqrt(
        heading.x * heading.x + heading.y * heading.y
    );

    if (headingLength <= 0.2f && globalGrid && globalPath.size() > 1)
    {
        std::size_t nearestPathIndex = 0;
        float nearestDistanceSquared =
            std::numeric_limits<float>::infinity();

        for (std::size_t i = 0; i < globalPath.size(); ++i)
        {
            const Vector2 point =
                globalGrid->CellToWorld(globalPath[i]);
            const float dx = point.x - robotPosition.x;
            const float dz = point.y - robotPosition.z;
            const float distanceSquared = dx * dx + dz * dz;

            if (distanceSquared < nearestDistanceSquared)
            {
                nearestDistanceSquared = distanceSquared;
                nearestPathIndex = i;
            }
        }

        if (nearestPathIndex + 1 < globalPath.size())
        {
            const Vector2 next =
                globalGrid->CellToWorld(globalPath[nearestPathIndex + 1]);
            heading = {
                next.x - robotPosition.x,
                next.y - robotPosition.z
            };
            headingLength = std::sqrt(
                heading.x * heading.x + heading.y * heading.y
            );
        }
    }

    if (headingLength > epsilon)
    {
        heading.x /= headingLength;
        heading.y /= headingLength;
    }

    struct GlobalRouteSample
    {
        float x = 0.0f;
        float z = 0.0f;
        float arrivalTime = 0.0f;
    };

    std::vector<GlobalRouteSample> routeSamples;
    if (globalGrid && globalPath.size() > 1)
    {
        std::size_t nearestPathIndex = 0;
        float nearestDistanceSquared =
            std::numeric_limits<float>::infinity();

        for (std::size_t i = 0; i < globalPath.size(); ++i)
        {
            const Vector2 point =
                globalGrid->CellToWorld(globalPath[i]);
            const float dx = point.x - robotPosition.x;
            const float dz = point.y - robotPosition.z;
            const float distanceSquared = dx * dx + dz * dz;

            if (distanceSquared < nearestDistanceSquared)
            {
                nearestDistanceSquared = distanceSquared;
                nearestPathIndex = i;
            }
        }

        Vector2 previousPosition = {
            robotPosition.x,
            robotPosition.z
        };
        float pathDistance = 0.0f;
        bool routeHorizonReached = false;

        for (std::size_t i = nearestPathIndex;
             i < globalPath.size() && !routeHorizonReached;
             ++i)
        {
            const Vector2 endpoint =
                globalGrid->CellToWorld(globalPath[i]);
            const float dx = endpoint.x - previousPosition.x;
            const float dz = endpoint.y - previousPosition.y;
            const float segmentLength =
                std::sqrt(dx * dx + dz * dz);
            const int samples = std::max(
                1,
                static_cast<int>(std::ceil(segmentLength / 0.5f))
            );

            for (int sample = 1; sample <= samples; ++sample)
            {
                const float alpha =
                    static_cast<float>(sample) / samples;
                const float arrivalTime =
                    (pathDistance + segmentLength * alpha) /
                    planningSpeed;

                if (arrivalTime > routeHorizon)
                {
                    routeHorizonReached = true;
                    break;
                }

                const float x = previousPosition.x + dx * alpha;
                const float z = previousPosition.y + dz * alpha;
                GridCell localCell;
                if (localGrid.WorldToCell(x, z, localCell))
                    routeSamples.push_back({ x, z, arrivalTime });
            }

            pathDistance += segmentLength;
            previousPosition = endpoint;
        }
    }

    for (const CachedObstacleTrajectory& trajectory : trajectories)
    {
        if (trajectory.samples.empty())
            continue;

        float obstacleTtc = -1.0f;
        bool threatensRobot = false;
        bool threatensFromBehind = false;
        PredictedObstaclePoint currentObstacle;
        if (!GetCachedObstacleAtTime(
            trajectory,
            cacheAge,
            currentObstacle))
        {
            continue;
        }

        const float relativeX =
            currentObstacle.position.x - robotPosition.x;
        const float relativeZ =
            currentObstacle.position.z - robotPosition.z;
        threatensFromBehind =
            headingLength > epsilon &&
            relativeX * heading.x + relativeZ * heading.y < 0.0f;

        float previousClearance =
            DistanceToObstacle(
                robotPosition.x,
                robotPosition.z,
                currentObstacle
            ) - robot.GetRadius();
        float previousTime = 0.0f;

        if (previousClearance <= 0.0f)
        {
            threatensRobot = true;
            obstacleTtc = 0.0f;
        }

        const float availableHorizon = std::max(
            0.0f,
            std::min(
                reactionHorizon,
                trajectory.samples.back().time - cacheAge
            )
        );

        for (float time = sampleStep;
             !threatensRobot && time <= availableHorizon + epsilon;
             time += sampleStep)
        {
            PredictedObstaclePoint predicted;
            if (!GetCachedObstacleAtTime(
                trajectory,
                cacheAge + time,
                predicted))
            {
                break;
            }

            const float robotX =
                robotPosition.x + robotVelocity.x * time;
            const float robotZ =
                robotPosition.z + robotVelocity.y * time;
            const float clearance =
                DistanceToObstacle(robotX, robotZ, predicted) -
                robot.GetRadius();

            const float closingDistance =
                previousClearance - clearance;

            if (clearance <= threatClearance &&
                (closingDistance > epsilon ||
                 clearance <= 0.0f))
            {
                threatensRobot = true;
                obstacleTtc = previousClearance <= threatClearance
                    ? previousTime
                    : previousTime +
                        (previousClearance - threatClearance) /
                        std::max(epsilon, closingDistance) *
                        sampleStep;
                break;
            }

            previousClearance = clearance;
            previousTime = time;
        }

        if (threatensRobot)
        {
            assessment.robotThreatened = true;
            if (threatensFromBehind)
                ++assessment.rearThreatsDetected;
        }

        float routeTtc = -1.0f;
        if (std::isfinite(cacheAge))
        {
            for (const GlobalRouteSample& sample : routeSamples)
            {
                PredictedObstaclePoint predicted;
                if (!GetCachedObstacleAtTime(
                    trajectory,
                    cacheAge + sample.arrivalTime,
                    predicted))
                {
                    continue;
                }

                if (DistanceToObstacle(
                    sample.x,
                    sample.z,
                    predicted) <= threatClearance)
                {
                    routeTtc = sample.arrivalTime;
                    assessment.globalPathThreatened = true;
                    break;
                }
            }
        }

        if (!threatensRobot && routeTtc < 0.0f)
            continue;

        ++assessment.threatsDetected;
        const float obstacleThreatTtc = threatensRobot
            ? obstacleTtc
            : routeTtc;

        if (assessment.minimumTimeToCollision < 0.0f ||
            obstacleThreatTtc < assessment.minimumTimeToCollision)
        {
            assessment.minimumTimeToCollision = obstacleThreatTtc;
            assessment.mostDangerousObstacleId =
                trajectory.obstacleId;
        }
    }

    return assessment;
}


bool NavigationController::IsTemporalPathSafe(
    const Robot& robot
) const
{
    if (!temporalPathValid || temporalPath.empty())
        return false;

    const auto& trajectories =
        localGrid.GetCachedObstacleTrajectories();
    const float cacheAge = localGrid.GetPredictionCacheAge();
    const float robotRadius = robot.GetRadius();
    const Vector3 robotPosition = robot.GetPosition();
    Vector2 previousPosition = {
        robotPosition.x,
        robotPosition.z
    };
    float previousTime = 0.0f;

    for (const TemporalWaypoint& waypoint : temporalPath)
    {
        const float relativeTime = std::max(
            0.0f,
            waypoint.arrivalTime - temporalPathElapsed
        );
        if (relativeTime + 0.0001f < previousTime)
            continue;

        const Vector2 endpoint =
            localGrid.CellToWorld(waypoint.cell);
        const float dx = endpoint.x - previousPosition.x;
        const float dz = endpoint.y - previousPosition.y;
        const float distance = std::sqrt(dx * dx + dz * dz);
        const int samples = std::max(
            1,
            static_cast<int>(std::ceil(
                distance / std::max(0.1f, localGrid.GetCellSize() * 0.25f)
            ))
        );

        for (int sample = 0; sample <= samples; ++sample)
        {
            const float alpha =
                static_cast<float>(sample) / samples;
            const float x = previousPosition.x + dx * alpha;
            const float z = previousPosition.y + dz * alpha;
            const float time =
                previousTime + (relativeTime - previousTime) * alpha;

            if (!localGrid.IsStaticFootprintFree(
                x,
                z,
                robotRadius))
            {
                return false;
            }

            for (const CachedObstacleTrajectory& trajectory :
                 trajectories)
            {
                PredictedObstaclePoint obstacle;
                if (!GetCachedObstacleAtTime(
                    trajectory,
                    cacheAge + time,
                    obstacle) ||
                    DistanceToObstacle(x, z, obstacle) <= robotRadius)
                {
                    return false;
                }
            }
        }

        previousPosition = endpoint;
        previousTime = relativeTime;
    }

    return true;
}
