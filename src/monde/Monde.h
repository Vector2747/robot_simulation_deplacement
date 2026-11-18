#pragma once

#include "raylib.h"
#include "Obstacle.h"
#include "../core/Config.h"

#include <vector>
#include <cstdint>

class World
{
private:
    TerrainConfig terrain;
    std::vector<Obstacle> obstacles;

    std::uint32_t nextObstacleId = 1;

    std::uint32_t AllocateObstacleId();
    std::uint64_t navigationRevision = 0;

public:
    explicit World(const TerrainConfig& terrainConfig);

    void GenerateObstacles(
        const ObstacleGenerationConfig& settings,
        const RobotConfig& robotConfig,
        Vector3 robotPosition,
        Vector3 targetPosition
    );

    void Update(float dt);

    void SetTerrain(const TerrainConfig& terrainConfig);

    std::uint32_t AddObstacle(const Obstacle& obstacle);
    bool RemoveObstacle(std::uint32_t id);

    const std::vector<Obstacle>& GetObstacles() const;

    const TerrainConfig& GetTerrain() const;

    // Prédit la position d'un obstacle sans modifier le monde.
    // Prédit l'état complet d'un obstacle sans modifier le monde.
    Obstacle GetPredictedObstacleState(
        const Obstacle& obstacle,
        float futureTime
    ) const;

    std::uint64_t GetNavigationRevision() const;
};