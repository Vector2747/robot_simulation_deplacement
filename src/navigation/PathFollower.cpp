#include "PathFollower.h"

#include <algorithm>
#include <cmath>

// ============================================
// CALCUL DE LA VITESSE DESIREE
// ============================================

Vector2 PathFollower::CalculateDesiredVelocity(
    const Robot& robot,
    const Target& target,
    const GridMap& grid,
    const std::vector<GridCell>& path,
    float arrivalRadius
) const
{
    // ----------------------------------------
    // 1. Vérification des paramètres
    // ----------------------------------------

    if (path.empty() ||
        !std::isfinite(arrivalRadius) ||
        !std::isfinite(waypointTolerance) ||
        !std::isfinite(speedGain))
    {
        return { 0.0f, 0.0f };
    }

    const Vector3 robotPosition = robot.GetPosition();
    const Vector3 targetPosition = target.GetPosition();

    // ----------------------------------------
    // 2. Vérification de l'arrivée
    // ----------------------------------------

    const float targetDx =
        targetPosition.x - robotPosition.x;

    const float targetDz =
        targetPosition.z - robotPosition.z;

    const float targetDistance = std::sqrt(
        targetDx * targetDx +
        targetDz * targetDz
    );

    if (!std::isfinite(targetDistance) ||
        targetDistance <= std::max(0.0f, arrivalRadius))
    {
        return { 0.0f, 0.0f };
    }

    // ----------------------------------------
    // 3. Recherche du prochain waypoint
    // ----------------------------------------

    Vector2 waypoint = {
        targetPosition.x,
        targetPosition.z
    };

    bool waypointFound = false;

    // On commence à 1 : la cellule 0 est la cellule
    // de départ du robot. Il ne doit pas y retourner.
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        const Vector2 candidate = grid.CellToWorld(path[i]);

        const float dx = candidate.x - robotPosition.x;
        const float dz = candidate.y - robotPosition.z;

        const float distance = std::sqrt(
            dx * dx + dz * dz
        );

        if (distance > waypointTolerance)
        {
            waypoint = candidate;
            waypointFound = true;
            break;
        }
    }

    // ----------------------------------------
    // 4. Direction vers le waypoint
    // ----------------------------------------

    float dx = waypoint.x - robotPosition.x;
    float dz = waypoint.y - robotPosition.z;

    float distance = std::sqrt(
        dx * dx + dz * dz
    );

    if (distance <= 0.0001f)
    {
        return { 0.0f, 0.0f };
    }

    // ----------------------------------------
    // 5. Calcul de la vitesse
    // ----------------------------------------

    const float directionX = dx / distance;
    const float directionZ = dz / distance;

    const float maxSpeed = robot.GetMaxSpeed();

    if (!std::isfinite(maxSpeed) || maxSpeed <= 0.0f)
    {
        return { 0.0f, 0.0f };
    }

    const float desiredSpeed = std::min(
        maxSpeed,
        targetDistance * speedGain
    );

    return {
        directionX * desiredSpeed,
        directionZ * desiredSpeed
    };
}