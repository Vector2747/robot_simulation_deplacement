
#pragma once

#include "raylib.h"

#include "LocalGrid.h"
#include "../monde/Monde.h"

#include <cstdint>
#include <vector>

struct LocalPlannerDiagnostics
{
    std::uint64_t evaluatedTrajectories = 0;
    std::uint64_t checkedSegments = 0;
    std::uint64_t rejectedSegments = 0;
    float selectedSpeed = 0.0f;
    bool selectedSafePrefix = false;
    int safePrefixSegments = 0;
    int preferredAvoidanceSide = 0;
    int selectedAvoidanceSide = 0;
};

struct VelocityCandidate
{
    Vector2 velocity = {0.0f, 0.0f};

    float score = 0.0f;

    bool isSafe = false;
};

class LocalPlanner
{
private:

    // Nombre de vitesses candidates testées.
    int velocitySamples = 24;

    // Durée de prédiction du déplacement du robot.
    float predictionHorizon = 3.0f;//1.0f;

    // Intervalle entre deux simulations.
    float predictionTimeStep = 0.1f;

    // Vitesse maximale du robot.
    float maxSpeed = 3.0f;

    // Rayon de sécurité du robot.
    float robotRadius = 0.0;//0.65f;

    // Liste des vitesses candidates.
    std::vector<VelocityCandidate> candidates;

    // Dernière vitesse sélectionnée par le contrôleur.
    Vector2 previousVelocity = { 0.0f, 0.0f };
    int preferredAvoidanceSide = 0;
    float preferredSideHoldTime = 0.0f;
    LocalPlannerDiagnostics lastDiagnostics;

public:

    LocalPlanner() = default;

    void Configure(
        float maximumSpeed,
        float radius
    );

    const LocalPlannerDiagnostics& GetLastDiagnostics() const
    {
        return lastDiagnostics;
    }

    void ResetMotionState()
    {
        previousVelocity = { 0.0f, 0.0f };
        preferredAvoidanceSide = 0;
        preferredSideHoldTime = 0.0f;
        lastDiagnostics = {};
    }

    
    Vector2 CalculateVelocity(
        const Vector3& robotPosition,
        const Vector2& desiredVelocity,
        const Vector3& targetPosition,
        const std::vector<GridCell>& globalPath,
        const GridMap& globalGrid,
        float dt,
        const LocalGrid& localGrid,
        const World& world,
        bool dynamicThreatActive = false,
        bool holdPositionWithoutGlobalPath = false
    );


};
