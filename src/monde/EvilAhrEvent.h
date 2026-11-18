#pragma once

#include "../core/Config.h"
#include "../navigation/NavigationController.h"
#include "../robot/Robot.h"
#include "../cible/Cible.h"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

struct EvilAhrConfig
{
    bool enabled = false;
    float annoyancePercent = 50.0f;
    float spawnFrequencySeconds = 8.0f;
    int maximumActiveObstacles = 20;
    float movingObstaclePercent = 50.0f;
    float lifetimeSeconds = 20.0f;
};

struct EvilAhrDiagnostics
{
    std::uint64_t wavesTriggered = 0;
    std::uint64_t obstaclesSpawned = 0;
    std::uint64_t obstaclesRemoved = 0;
    int activeOwnedObstacles = 0;
    int lastWaveSpawned = 0;
    std::string lastEvent = "En attente";
};

class EvilAhrEvent
{
private:
    struct ManagedObstacle
    {
        std::uint32_t id = 0;
        float remainingLifetime = 0.0f;
    };

    EvilAhrConfig config;
    EvilAhrDiagnostics diagnostics;
    std::vector<ManagedObstacle> managedObstacles;
    std::mt19937 randomGenerator;

    float spawnAccumulator = 0.0f;
    float maintenanceAccumulator = 0.0f;
    bool waveRequested = false;
    std::uint64_t observedWorldRevision = 0;

    void PruneExternallyRemovedObstacles(const World& world);
    int SpawnWave(
        World& world,
        const Robot& robot,
        const Target& target,
        const NavigationController& navigation,
        const ObstacleGenerationConfig& obstacleSettings
    );
    bool SpawnOne(
        World& world,
        const Robot& robot,
        const Target& target,
        const NavigationController& navigation,
        const ObstacleGenerationConfig& obstacleSettings
    );

public:
    EvilAhrEvent();
    explicit EvilAhrEvent(std::uint32_t randomSeed);

    EvilAhrConfig& GetConfig();
    const EvilAhrConfig& GetConfig() const;
    const EvilAhrDiagnostics& GetDiagnostics() const;

    void RequestWave();
    void Reset(World& world);
    void Update(
        World& world,
        const Robot& robot,
        const Target& target,
        const NavigationController& navigation,
        const ObstacleGenerationConfig& obstacleSettings,
        float dt
    );
};
