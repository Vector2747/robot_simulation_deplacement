#pragma once

#include "LocalGrid.h"
#include "../planification/GridMap.h"

#include <vector>
#include <cstdint>

struct TemporalWaypoint
{
    GridCell cell;
    float arrivalTime = 0.0f;
};

struct TemporalAStarConfig
{
    float timeStep = 0.1f;
    float predictionHorizon = 3.0f;

    // Vitesse prudente utilisée pour estimer les temps de déplacement.
    float planningSpeed = 1.8f;

    int maxExpandedNodes = 4000;

    float waitCost = 0.1f;
    float diagonalCost = 1.41421356f;

    float robotRadius = 0.65f;
};

struct TemporalAStarSearchDiagnostics
{
    int expandedNodes = 0;
    int rejectedNeighbors = 0;
    int rejectedCollisions = 0;
    int rejectedTimeHorizon = 0;
    int rejectedLocalWindow = 0;
    int rejectedStaticObstacles = 0;
    int statesWithoutSuccessor = 0;
    int deepestTimeLayer = 0;
};

class TemporalAStar
{
private:
    TemporalAStarConfig config;

    // Statistiques de la dernière recherche.
    mutable int lastExpandedNodes = 0;
    mutable bool hasSearched = false;
    mutable bool lastSearchSucceeded = false;

    enum class FailureReason
    {
        None,
        GridNotInitialized,
        InvalidConfig,
        InvalidDimensions,
        InvalidCoordinates,
        MissingTrajectoryCache,
        MissingRelevantTrajectory,
        InvalidTimeHorizon,
        StartBlocked,
        GoalUnreachable,
        NoValidSuccessor,
        SearchLimitReached,
        PredictionHorizonExceeded,
        LocalWindowExceeded,
        ReconstructionFailed
    };

    mutable FailureReason lastFailureReason = FailureReason::None;
    mutable TemporalAStarSearchDiagnostics lastDiagnostics;

public:
    TemporalAStar() = default;

    void Configure(const TemporalAStarConfig& newConfig);

    bool FindPath(
        const LocalGrid& localGrid,
        const GridCell& start,
        const GridCell& goal,
        std::vector<TemporalWaypoint>& outputPath
    ) const;

    bool FindPath(
        const LocalGrid& localGrid,
        const GridCell& start,
        const std::vector<GridCell>& acceptableGoals,
        std::vector<TemporalWaypoint>& outputPath
    ) const;

    int GetLastExpandedNodes() const
    {
        return lastExpandedNodes;
    }

    bool HasSearched() const
    {
        return hasSearched;
    }

    bool WasLastSearchSuccessful() const
    {
        return lastSearchSucceeded;
    }

    const TemporalAStarSearchDiagnostics& GetLastDiagnostics() const
    {
        return lastDiagnostics;
    }

    const char* GetLastFailureReason() const
    {
        switch (lastFailureReason)
        {
        case FailureReason::None: return "AUCUN";
        case FailureReason::GridNotInitialized: return "GRILLE NON INITIALISEE";
        case FailureReason::InvalidConfig: return "CONFIGURATION INVALIDE";
        case FailureReason::InvalidDimensions: return "DIMENSIONS INVALIDES";
        case FailureReason::InvalidCoordinates: return "COORDONNEES INVALIDES";
        case FailureReason::MissingTrajectoryCache: return "CACHE TRAJECTOIRES ABSENT";
        case FailureReason::MissingRelevantTrajectory: return "TRAJECTOIRE MANQUANTE";
        case FailureReason::InvalidTimeHorizon: return "HORIZON TEMPOREL INVALIDE";
        case FailureReason::StartBlocked: return "StartBlocked";
        case FailureReason::GoalUnreachable: return "GoalUnreachable";
        case FailureReason::NoValidSuccessor: return "NoValidSuccessor";
        case FailureReason::SearchLimitReached: return "SearchLimitReached";
        case FailureReason::PredictionHorizonExceeded: return "PredictionHorizonExceeded";
        case FailureReason::LocalWindowExceeded: return "LocalWindowExceeded";
        case FailureReason::ReconstructionFailed: return "ReconstructionFailed";
        }

        return "INCONNU";
    }
    
};