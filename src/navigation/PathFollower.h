#pragma once

#include "../robot/Robot.h"
#include "../cible/Cible.h"
#include "../planification/GridMap.h"

#include <vector>

class PathFollower
{
private:
    // Distance à partir de laquelle un waypoint
    // est considéré comme atteint.
    float waypointTolerance = 0.35f;

    // Gain utilisé pour ralentir à proximité
    // d'un waypoint.
    float speedGain = 2.0f;

public:
    Vector2 CalculateDesiredVelocity(
        const Robot& robot,
        const Target& target,
        const GridMap& grid,
        const std::vector<GridCell>& path,
        float arrivalRadius
    ) const;
};