#pragma once

#include "../core/NavigationState.h"
#include "../robot/Robot.h"
#include "../cible/Cible.h"
#include "../core/Config.h"
#include "../monde/Monde.h"

#include "../planification/GridMap.h"
#include "../planification/GridSynchronizer.h"
#include "../planification/DStarLite.h"
#include "LocalGrid.h"

#include "PathFollower.h"
#include "LocalPlanner.h"
#include "TemporalAStar.h"

#include <memory>
#include <cstdint>
#include <string>
#include <vector>

struct TemporalPlanningDiagnostics
{
    int threatsDetected = 0;
    int rearThreatsDetected = 0;
    float minimumTimeToCollision = -1.0f;
    std::uint32_t mostDangerousObstacleId = 0;
    std::string lastTriggerReason = "AUCUNE";
    std::uint64_t searchesRun = 0;
    std::uint64_t searchesSucceeded = 0;
    std::uint64_t searchesFailed = 0;
    int lastExpandedNodes = 0;
    int lastRejectedNeighbors = 0;
    int lastRejectedCollisions = 0;
    int lastRejectedTimeHorizon = 0;
    int lastRejectedLocalWindow = 0;
    std::uint64_t totalExpandedNodes = 0;
    float lastSearchDurationMilliseconds = 0.0f;
    std::vector<float> recentSearchDurationsMilliseconds;
    std::string lastSearchFailureReason = "AUCUNE";
};

class NavigationController
{
private:
    struct TemporalThreatAssessment
    {
        int threatsDetected = 0;
        int rearThreatsDetected = 0;
        float minimumTimeToCollision = -1.0f;
        std::uint32_t mostDangerousObstacleId = 0;
        bool robotThreatened = false;
        bool globalPathThreatened = false;

        bool IsThreatened() const
        {
            return threatsDetected > 0;
        }
    };

    NavigationState state = NavigationState::Idle;

    // Carte globale.
    std::unique_ptr<GridMap> globalGrid;

    // Synchronisation du monde avec la carte.
    GridSynchronizer gridSynchronizer;

    // Planificateur global.
    DStarLite globalPlanner;

    // Dernières cellules de départ et d'arrivée.
    GridCell lastStartCell;
    GridCell lastGoalCell;

    bool hasStartCell = false;
    bool hasGoalCell = false;

    // Chemin global calculé.
    std::vector<GridCell> globalPath;

    // Suivi du chemin global.
    PathFollower pathFollower;

    // Grille locale autour du robot.
    LocalGrid localGrid;

    LocalPlanner localPlanner;

    // Planification locale spatio-temporelle
    TemporalAStar temporalPlanner;

    std::vector<TemporalWaypoint> temporalPath;

    bool temporalPathValid = false;
    bool temporalPlanningRequested = false;
    bool previousTemporalThreat = false;
    int lastTemporalTriggerThreatCount = 0;
    float lastTemporalTriggerTtc = -1.0f;
    std::string pendingTemporalTriggerReason;

    TemporalPlanningDiagnostics temporalDiagnostics;

    bool FindTemporalGoals(
        const Robot& robot,
        std::vector<GridCell>& outputGoals
    ) const;

    TemporalThreatAssessment AssessTemporalThreats(
        const Robot& robot,
        float reactionHorizon = 2.5f
    ) const;

    bool IsTemporalPathSafe(const Robot& robot) const;
    void ConfigureMotionLimits(const Robot& robot);

    // Limitation de fréquence des recherches temporelles.
    float temporalPlanningAccumulator = 0.0f;
    float temporalPlanningInterval = 0.25f;
    float noPathThreatGraceRemaining = 0.0f;

    // Temps écoulé depuis le début du plan temporel.
    float temporalPathElapsed = 0.0f;

    // Waypoint temporel actuellement suivi.
    std::size_t temporalPathIndex = 0;

public:
    bool Initialize(
        const World& world,
        const Robot& robot,
        const Target& target
    );

    void Update(
        const World& world,
        const Robot& robot,
        const Target& target,
        const NavigationConfig& config,
        float dt
    );

    NavigationState GetState() const;

    float GetDistanceToTarget(
        const Robot& robot,
        const Target& target
    ) const;

    const std::vector<GridCell>& GetGlobalPath() const;

    const GridMap* GetGlobalGrid() const;

    Vector2 CalculateDesiredVelocity(
        const Robot& robot,
        const Target& target,
        const float dt,
        const NavigationConfig& config,
        const World& world
    );

    const LocalGrid* GetLocalGrid() const;

    void SetStableWindowMovement(bool enabled)
    {
        localGrid.SetStableWindowMovement(enabled);
    }

    bool IsStableWindowMovement() const
    {
        return localGrid.IsStableWindowMovement();
    }

    bool IsTemporalPathActive() const
    {
        return temporalPathValid && !temporalPath.empty();
    }

    const std::vector<TemporalWaypoint>& GetTemporalPath() const
    {
        return temporalPath;
    }

    int GetTemporalExpandedNodes() const
    {
        return temporalPlanner.GetLastExpandedNodes();
    }

    bool HasTemporalSearchRun() const
    {
        return temporalPlanner.HasSearched();
    }

    bool WasLastTemporalSearchSuccessful() const
    {
        return temporalPlanner.WasLastSearchSuccessful();
    }

    const TemporalPlanningDiagnostics& GetTemporalDiagnostics() const
    {
        return temporalDiagnostics;
    }

    const LocalPlannerDiagnostics& GetLocalPlannerDiagnostics() const
    {
        return localPlanner.GetLastDiagnostics();
    }

    const char* GetTemporalFailureReason() const
    {
        return temporalPlanner.GetLastFailureReason();
    }
};