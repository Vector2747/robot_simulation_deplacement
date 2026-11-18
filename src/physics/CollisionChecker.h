
#pragma once

#include "raylib.h"
#include "../core/Config.h"
#include "../monde/Obstacle.h"

#include <vector>

class CollisionChecker
{
public:
    // Cercle du robot contre un obstacle rectangulaire.
    static bool CircleIntersectsObstacle(
        Vector3 center,
        float radius,
        const Obstacle& obstacle
    );

    // Vérifie une position contre tous les obstacles.
    static bool IsPositionFree(
        Vector3 position,
        float radius,
        const std::vector<Obstacle>& obstacles
    );

    // Vérifie que le cercle reste entièrement dans le terrain.
    static bool IsInsideTerrain(
        Vector3 position,
        float radius,
        const TerrainConfig& terrain
    );

    // Vérification combinée : terrain et obstacles.
    static bool IsPositionSafe(
        Vector3 position,
        float radius,
        const TerrainConfig& terrain,
        const std::vector<Obstacle>& obstacles
    );

    // Vérifie l'intégralité d'un déplacement.
    static bool IsMovementSafe(
        Vector3 start,
        Vector3 end,
        float radius,
        const TerrainConfig& terrain,
        const std::vector<Obstacle>& obstacles
    );
};
