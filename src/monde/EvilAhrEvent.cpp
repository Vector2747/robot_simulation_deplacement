#include "EvilAhrEvent.h"

#include "../planification/GridMap.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    constexpr int MAXIMUM_WORLD_OBSTACLES = 200;
    constexpr float MAINTENANCE_INTERVAL = 0.25f;

    float ClampFinite(float value, float minimum, float maximum)
    {
        if (!std::isfinite(value))
            return minimum;
        return std::clamp(value, minimum, maximum);
    }

    float DistanceSquared(float x1, float z1, float x2, float z2)
    {
        const float dx = x1 - x2;
        const float dz = z1 - z2;
        return dx * dx + dz * dz;
    }

    float DistanceToRectangle(
        float x,
        float z,
        const Vector3& center,
        const Vector3& size
    )
    {
        const float closestX = std::clamp(
            x,
            center.x - size.x * 0.5f,
            center.x + size.x * 0.5f
        );
        const float closestZ = std::clamp(
            z,
            center.z - size.z * 0.5f,
            center.z + size.z * 0.5f
        );
        return std::sqrt(DistanceSquared(x, z, closestX, closestZ));
    }

    bool RectanglesOverlap(
        const Vector3& firstPosition,
        const Vector3& firstSize,
        const Vector3& secondPosition,
        const Vector3& secondSize,
        float clearance
    )
    {
        return
            std::abs(firstPosition.x - secondPosition.x) <
                (firstSize.x + secondSize.x) * 0.5f + clearance &&
            std::abs(firstPosition.z - secondPosition.z) <
                (firstSize.z + secondSize.z) * 0.5f + clearance;
    }
}

EvilAhrEvent::EvilAhrEvent()
    : randomGenerator(std::random_device{}())
{
}

EvilAhrEvent::EvilAhrEvent(std::uint32_t randomSeed)
    : randomGenerator(randomSeed)
{
}

EvilAhrConfig& EvilAhrEvent::GetConfig()
{
    return config;
}

const EvilAhrConfig& EvilAhrEvent::GetConfig() const
{
    return config;
}

const EvilAhrDiagnostics& EvilAhrEvent::GetDiagnostics() const
{
    return diagnostics;
}

void EvilAhrEvent::RequestWave()
{
    waveRequested = true;
}

void EvilAhrEvent::Reset(World& world)
{
    for (const ManagedObstacle& obstacle : managedObstacles)
    {
        if (world.RemoveObstacle(obstacle.id))
            ++diagnostics.obstaclesRemoved;
    }

    managedObstacles.clear();
    spawnAccumulator = 0.0f;
    maintenanceAccumulator = 0.0f;
    waveRequested = false;
    diagnostics = {};
    diagnostics.lastEvent = "Evenement reinitialise";
    observedWorldRevision = world.GetNavigationRevision();
}

void EvilAhrEvent::PruneExternallyRemovedObstacles(const World& world)
{
    const std::uint64_t revision = world.GetNavigationRevision();
    if (revision == observedWorldRevision)
        return;

    std::vector<std::uint32_t> worldObstacleIds;
    worldObstacleIds.reserve(world.GetObstacles().size());
    for (const Obstacle& obstacle : world.GetObstacles())
        worldObstacleIds.push_back(obstacle.id);

    managedObstacles.erase(
        std::remove_if(
            managedObstacles.begin(),
            managedObstacles.end(),
            [&](const ManagedObstacle& managed)
            {
                const bool stillExists = std::find(
                    worldObstacleIds.begin(),
                    worldObstacleIds.end(),
                    managed.id
                ) != worldObstacleIds.end();

                if (!stillExists)
                    ++diagnostics.obstaclesRemoved;
                return !stillExists;
            }
        ),
        managedObstacles.end()
    );

    observedWorldRevision = revision;
}

void EvilAhrEvent::Update(
    World& world,
    const Robot& robot,
    const Target& target,
    const NavigationController& navigation,
    const ObstacleGenerationConfig& obstacleSettings,
    float dt
)
{
    if (!std::isfinite(dt) || dt < 0.0f)
        return;

    config.annoyancePercent = ClampFinite(
        config.annoyancePercent,
        0.0f,
        100.0f
    );
    config.spawnFrequencySeconds = ClampFinite(
        config.spawnFrequencySeconds,
        0.25f,
        120.0f
    );
    config.maximumActiveObstacles = std::clamp(
        config.maximumActiveObstacles,
        1,
        MAXIMUM_WORLD_OBSTACLES
    );
    config.movingObstaclePercent = ClampFinite(
        config.movingObstaclePercent,
        0.0f,
        100.0f
    );
    config.lifetimeSeconds = ClampFinite(
        config.lifetimeSeconds,
        0.25f,
        600.0f
    );

    if (config.enabled)
        spawnAccumulator += dt;
    maintenanceAccumulator += dt;

    const float difficulty = config.annoyancePercent / 100.0f;
    const float effectiveFrequency =
        config.spawnFrequencySeconds * (1.5f - 1.15f * difficulty);
    const bool automaticWaveDue =
        config.enabled && spawnAccumulator >= effectiveFrequency;
    const bool manualWaveDue = waveRequested;
    waveRequested = false;

    const bool maintenanceDue =
        maintenanceAccumulator >= MAINTENANCE_INTERVAL;
    if (!maintenanceDue && !automaticWaveDue && !manualWaveDue)
        return;

    const float maintenanceElapsed = maintenanceAccumulator;
    maintenanceAccumulator = 0.0f;
    PruneExternallyRemovedObstacles(world);

    std::uniform_real_distribution<float> unitDistribution(0.0f, 1.0f);
    const float randomRemovalRate = 0.025f + 0.20f * difficulty;
    const float randomRemovalProbability =
        1.0f - std::exp(-randomRemovalRate * maintenanceElapsed);

    for (std::size_t i = 0; i < managedObstacles.size();)
    {
        ManagedObstacle& managed = managedObstacles[i];
        managed.remainingLifetime -= maintenanceElapsed;
        const bool lifetimeExpired = managed.remainingLifetime <= 0.0f;
        const bool disappearedRandomly =
            config.enabled &&
            unitDistribution(randomGenerator) < randomRemovalProbability;

        if (!lifetimeExpired && !disappearedRandomly)
        {
            ++i;
            continue;
        }

        const bool removed = world.RemoveObstacle(managed.id);
        if (removed)
            ++diagnostics.obstaclesRemoved;
        managedObstacles.erase(managedObstacles.begin() + i);
        diagnostics.lastEvent = lifetimeExpired
            ? "Obstacle expire"
            : "Disparition aleatoire";
    }
    observedWorldRevision = world.GetNavigationRevision();

    if (automaticWaveDue)
    {
        spawnAccumulator = std::fmod(
            spawnAccumulator,
            std::max(0.25f, effectiveFrequency)
        );
    }

    if (manualWaveDue || automaticWaveDue)
    {
        diagnostics.lastWaveSpawned = SpawnWave(
            world,
            robot,
            target,
            navigation,
            obstacleSettings
        );
    }

    diagnostics.activeOwnedObstacles =
        static_cast<int>(managedObstacles.size());
}

int EvilAhrEvent::SpawnWave(
    World& world,
    const Robot& robot,
    const Target& target,
    const NavigationController& navigation,
    const ObstacleGenerationConfig& obstacleSettings
)
{
    ++diagnostics.wavesTriggered;
    diagnostics.lastWaveSpawned = 0;

    const float difficulty = config.annoyancePercent / 100.0f;
    std::uniform_real_distribution<float> unitDistribution(0.0f, 1.0f);
    const int baseCount = 1 + static_cast<int>(difficulty * 3.0f);
    const int waveSize = std::min(
        5,
        baseCount + (unitDistribution(randomGenerator) < difficulty * 0.85f
            ? 1
            : 0)
    );

    for (int i = 0; i < waveSize; ++i)
    {
        const int worldLimit = std::min(
            MAXIMUM_WORLD_OBSTACLES,
            config.maximumActiveObstacles
        );
        if (static_cast<int>(world.GetObstacles().size()) >= worldLimit)
        {
            diagnostics.lastEvent = "Limite d'obstacles atteinte";
            break;
        }

        if (!SpawnOne(
            world,
            robot,
            target,
            navigation,
            obstacleSettings))
        {
            diagnostics.lastEvent = "Aucun emplacement sur";
            break;
        }

        ++diagnostics.lastWaveSpawned;
        ++diagnostics.obstaclesSpawned;
    }

    observedWorldRevision = world.GetNavigationRevision();
    if (diagnostics.lastWaveSpawned > 0)
        diagnostics.lastEvent = "Vague generee sur le chemin";

    return diagnostics.lastWaveSpawned;
}

bool EvilAhrEvent::SpawnOne(
    World& world,
    const Robot& robot,
    const Target& target,
    const NavigationController& navigation,
    const ObstacleGenerationConfig& obstacleSettings
)
{
    const TerrainConfig& terrain = world.GetTerrain();
    const float terrainWidth = terrain.cellsX * terrain.cellSize;
    const float terrainDepth = terrain.cellsZ * terrain.cellSize;
    if (!std::isfinite(terrainWidth) ||
        !std::isfinite(terrainDepth) ||
        terrainWidth <= 0.0f ||
        terrainDepth <= 0.0f)
    {
        return false;
    }

    const float maximumSize = std::max(
        0.5f,
        std::min(
            4.0f,
            std::isfinite(obstacleSettings.maxSize)
                ? obstacleSettings.maxSize
                : 1.0f
        )
    );
    const float minimumSize = std::clamp(
        std::isfinite(obstacleSettings.minSize)
            ? obstacleSettings.minSize
            : 1.0f,
        0.5f,
        maximumSize
    );
    std::uniform_real_distribution<float> sizeDistribution(
        minimumSize,
        std::max(minimumSize, maximumSize)
    );
    std::uniform_real_distribution<float> unitDistribution(0.0f, 1.0f);
    std::uniform_real_distribution<float> jitterDistribution(-0.2f, 0.2f);
    std::uniform_real_distribution<float> angleDistribution(
        0.0f,
        2.0f * PI
    );

    const Vector3 size = {
        sizeDistribution(randomGenerator),
        1.0f,
        sizeDistribution(randomGenerator)
    };

    const Vector3 robotPosition = robot.GetPosition();
    const Vector3 targetPosition = target.GetPosition();
    const float robotClearance =
        robot.GetRadius() +
        std::sqrt(size.x * size.x + size.z * size.z) * 0.5f +
        0.5f;
    const float targetClearance =
        robot.GetRadius() +
        std::sqrt(size.x * size.x + size.z * size.z) * 0.5f;

    const float difficulty = config.annoyancePercent / 100.0f;
    const GridMap* globalGrid = navigation.GetGlobalGrid();
    const std::vector<GridCell>& path = navigation.GetGlobalPath();

    std::vector<Vector2> candidateAnchors;
    Vector2 pathDirection = { 1.0f, 0.0f };
    if (globalGrid != nullptr && !path.empty())
    {
        std::size_t nearestPathIndex = 0;
        float nearestDistanceSquared =
            std::numeric_limits<float>::infinity();

        for (std::size_t i = 0; i < path.size(); ++i)
        {
            const Vector2 position = globalGrid->CellToWorld(path[i]);
            const float distanceSquared = DistanceSquared(
                position.x,
                position.y,
                robotPosition.x,
                robotPosition.z
            );
            if (distanceSquared < nearestDistanceSquared)
            {
                nearestDistanceSquared = distanceSquared;
                nearestPathIndex = i;
            }
        }

        const float maximumRouteDistance =
            10.0f + (1.0f - difficulty) * 35.0f;
        float routeDistance = 0.0f;
        Vector2 previous = {
            robotPosition.x,
            robotPosition.z
        };

        for (std::size_t i = nearestPathIndex;
             i < path.size() && routeDistance <= maximumRouteDistance;
             ++i)
        {
            const Vector2 position = globalGrid->CellToWorld(path[i]);
            routeDistance += std::sqrt(DistanceSquared(
                previous.x,
                previous.y,
                position.x,
                position.y
            ));

            if (routeDistance >= robotClearance &&
                std::sqrt(DistanceSquared(
                    robotPosition.x,
                    robotPosition.z,
                    position.x,
                    position.y
                )) >= robotClearance)
            {
                candidateAnchors.push_back(position);
            }

            if (i + 1 < path.size())
            {
                const Vector2 next =
                    globalGrid->CellToWorld(path[i + 1]);
                const float dx = next.x - position.x;
                const float dz = next.y - position.y;
                const float length = std::sqrt(dx * dx + dz * dz);
                if (length > 0.0001f)
                    pathDirection = { dx / length, dz / length };
            }

            previous = position;
        }
    }

    const TerrainConfig& worldTerrain = world.GetTerrain();
    const float minX = -terrainWidth * 0.5f + size.x * 0.5f;
    const float maxX = terrainWidth * 0.5f - size.x * 0.5f;
    const float minZ = -terrainDepth * 0.5f + size.z * 0.5f;
    const float maxZ = terrainDepth * 0.5f - size.z * 0.5f;
    if (minX > maxX || minZ > maxZ)
        return false;

    const float pathBias = 0.25f + 0.72f * difficulty;
    const float lateralRange = 0.6f + (1.0f - difficulty) * 2.5f;
    const float fallbackMinRadius = robotClearance;
    const float fallbackMaxRadius =
        fallbackMinRadius + 5.0f + (1.0f - difficulty) * 10.0f;

    for (int attempt = 0; attempt < 48; ++attempt)
    {
        Vector2 anchor = { 0.0f, 0.0f };
        if (!candidateAnchors.empty())
        {
            std::uniform_int_distribution<std::size_t> anchorDistribution(
                0,
                candidateAnchors.size() - 1
            );
            anchor = candidateAnchors[anchorDistribution(randomGenerator)];
        }
        else
        {
            const float angle = angleDistribution(randomGenerator);
            const float radius = std::uniform_real_distribution<float>(
                fallbackMinRadius,
                std::max(fallbackMinRadius, fallbackMaxRadius)
            )(randomGenerator);
            anchor = {
                robotPosition.x + std::cos(angle) * radius,
                robotPosition.z + std::sin(angle) * radius
            };
        }

        Vector2 candidate = anchor;
        if (unitDistribution(randomGenerator) >= pathBias)
        {
            const float offset = std::uniform_real_distribution<float>(
                0.4f,
                std::max(0.4f, lateralRange)
            )(randomGenerator);
            const float side =
                unitDistribution(randomGenerator) < 0.5f ? -1.0f : 1.0f;
            candidate.x += -pathDirection.y * offset * side;
            candidate.y += pathDirection.x * offset * side;
        }
        candidate.x += jitterDistribution(randomGenerator);
        candidate.y += jitterDistribution(randomGenerator);

        if (candidate.x < minX || candidate.x > maxX ||
            candidate.y < minZ || candidate.y > maxZ)
        {
            continue;
        }

        const Vector3 position = {
            candidate.x,
            worldTerrain.groundY + size.y * 0.5f,
            candidate.y
        };
        if (DistanceToRectangle(
            robotPosition.x,
            robotPosition.z,
            position,
            size) < robot.GetRadius() + 0.25f)
        {
            continue;
        }

        if (DistanceToRectangle(
            targetPosition.x,
            targetPosition.z,
            position,
            size) < targetClearance)
        {
            continue;
        }

        bool overlapsExistingObstacle = false;
        for (const Obstacle& obstacle : world.GetObstacles())
        {
            if (RectanglesOverlap(
                position,
                size,
                obstacle.position,
                obstacle.size,
                0.15f))
            {
                overlapsExistingObstacle = true;
                break;
            }
        }
        if (overlapsExistingObstacle)
            continue;

        const bool isMoving =
            unitDistribution(randomGenerator) <
            config.movingObstaclePercent / 100.0f;
        Vector2 velocity = { 0.0f, 0.0f };
        if (isMoving)
        {
            float angle = angleDistribution(randomGenerator);
            if (unitDistribution(randomGenerator) < 0.35f * difficulty)
            {
                angle = std::atan2(
                    robotPosition.z - position.z,
                    robotPosition.x - position.x
                );
            }

            const float minimumSpeed = std::max(
                0.5f,
                std::isfinite(obstacleSettings.minSpeed)
                    ? obstacleSettings.minSpeed
                    : 0.5f
            );
            const float maximumSpeed = std::max(
                minimumSpeed,
                std::isfinite(obstacleSettings.maxSpeed)
                    ? obstacleSettings.maxSpeed
                    : minimumSpeed
            );
            const float speed = std::uniform_real_distribution<float>(
                minimumSpeed,
                maximumSpeed
            )(randomGenerator);
            velocity = {
                std::cos(angle) * speed,
                std::sin(angle) * speed
            };
        }

        Obstacle obstacle;
        obstacle.position = position;
        obstacle.size = size;
        obstacle.isMoving = isMoving;
        obstacle.velocity = velocity;
        const std::uint32_t id = world.AddObstacle(obstacle);
        if (id == 0)
            return false;

        const float lifetimeScale = std::uniform_real_distribution<float>(
            0.65f,
            1.35f
        )(randomGenerator);
        const float difficultyLifetimeScale = 1.15f - 0.45f * difficulty;
        managedObstacles.push_back({
            id,
            std::max(
                0.25f,
                config.lifetimeSeconds *
                    lifetimeScale *
                    difficultyLifetimeScale
            )
        });
        return true;
    }

    return false;
}
