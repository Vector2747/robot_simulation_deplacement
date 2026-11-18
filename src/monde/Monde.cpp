
#include "Monde.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

std::uint32_t World::AllocateObstacleId()
{
    // 0 signifie que tous les identifiants ont été utilisés.
    if (nextObstacleId == 0)
        return 0;

    const std::uint32_t id = nextObstacleId;

    if (nextObstacleId == std::numeric_limits<std::uint32_t>::max())
        nextObstacleId = 0;
    else
        ++nextObstacleId;

    return id;
}

namespace
{
    // Générateur aléatoire partagé par les opérations de World.
    std::mt19937& GetRandomGenerator()
    {
        static std::mt19937 rng(std::random_device{}());
        return rng;
    }

    // Vérifie qu'une valeur flottante est exploitable.
    bool IsValidFloat(float value)
    {
        return std::isfinite(value);
    }

    // Garantit une configuration de terrain valide.
    TerrainConfig SanitizeTerrain(const TerrainConfig& config)
    {
        TerrainConfig result = config;

        if (result.cellsX <= 0)
            result.cellsX = 1;

        if (result.cellsZ <= 0)
            result.cellsZ = 1;

        if (!IsValidFloat(result.cellSize) ||
            result.cellSize <= 0.0f)
        {
            result.cellSize = 1.0f;
        }

        if (!IsValidFloat(result.groundY))
            result.groundY = 0.0f;

        return result;
    }

    
    struct TerrainBounds
    {
        float minX;
        float maxX;
        float minZ;
        float maxZ;
    };

    TerrainBounds GetTerrainBounds(const TerrainConfig& config)
    {
        const float width = config.cellsX * config.cellSize;
        const float depth = config.cellsZ * config.cellSize;

        return {
            -width * 0.5f,
             width * 0.5f,
            -depth * 0.5f,
             depth * 0.5f
        };
    }

    // Vérifie qu'un rectangle tient dans le terrain.
    bool FitsInsideTerrain(
        const Vector3& position,
        const Vector3& size,
        const TerrainBounds& bounds
    )
    {
        const float halfX = size.x * 0.5f;
        const float halfZ = size.z * 0.5f;

        return
            position.x - halfX >= bounds.minX &&
            position.x + halfX <= bounds.maxX &&
            position.z - halfZ >= bounds.minZ &&
            position.z + halfZ <= bounds.maxZ;
    }

    // Intersection entre un rectangle aligné sur les axes
    // et un cercle dans le plan X/Z.
    bool RectangleIntersectsCircle(
        const Vector3& rectanglePosition,
        const Vector3& rectangleSize,
        const Vector3& circlePosition,
        float radius
    )
    {
        const float halfX = rectangleSize.x * 0.5f;
        const float halfZ = rectangleSize.z * 0.5f;

        const float closestX = std::clamp(
            circlePosition.x,
            rectanglePosition.x - halfX,
            rectanglePosition.x + halfX
        );

        const float closestZ = std::clamp(
            circlePosition.z,
            rectanglePosition.z - halfZ,
            rectanglePosition.z + halfZ
        );

        const float dx = circlePosition.x - closestX;
        const float dz = circlePosition.z - closestZ;

        return dx * dx + dz * dz <= radius * radius;
    }

    // Vérifie si un obstacle évite les zones protégées.
    bool AvoidsProtectedZones(
        const Vector3& obstaclePosition,
        const Vector3& obstacleSize,
        const Vector3& robotPosition,
        const Vector3& targetPosition,
        float protectedRadius
    )
    {
        if (RectangleIntersectsCircle(
            obstaclePosition,
            obstacleSize,
            robotPosition,
            protectedRadius))
        {
            return false;
        }

        if (RectangleIntersectsCircle(
            obstaclePosition,
            obstacleSize,
            targetPosition,
            protectedRadius))
        {
            return false;
        }

        return true;
    }

}

// Constructeur
World::World(const TerrainConfig& terrainConfig)
    : terrain(SanitizeTerrain(terrainConfig))
{
}


void World::GenerateObstacles(
    const ObstacleGenerationConfig& settings,
    const RobotConfig& robotConfig,
    Vector3 robotPosition,
    Vector3 targetPosition
)
{
    // Une nouvelle génération remplace l'ancien environnement.
    obstacles.clear();
    ++navigationRevision;

    if (settings.count <= 0)
        return;

    const TerrainBounds bounds = GetTerrainBounds(terrain);

    const float terrainWidth = bounds.maxX - bounds.minX;
    const float terrainDepth = bounds.maxZ - bounds.minZ;

    if (terrainWidth <= 0.0f || terrainDepth <= 0.0f)
        return;

    const float minSize = std::max(
        0.1f,
        IsValidFloat(settings.minSize) ? settings.minSize : 0.1f
    );

    const float maxSize = std::max(
        minSize,
        IsValidFloat(settings.maxSize) ? settings.maxSize : minSize
    );

    const float minSpeed = std::max(
        0.0f,
        IsValidFloat(settings.minSpeed) ? settings.minSpeed : 0.0f
    );

    const float maxSpeed = std::max(
        minSpeed,
        IsValidFloat(settings.maxSpeed) ? settings.maxSpeed : minSpeed
    );

    const float movingProbability = std::clamp(
        IsValidFloat(settings.movingProbability)
            ? settings.movingProbability
            : 0.0f,
        0.0f,
        1.0f
    );

    const float protectedRadius = std::max(
        0.0f,
        IsValidFloat(robotConfig.radius) ? robotConfig.radius : 0.0f
    );

    constexpr int MAX_ATTEMPTS_PER_OBSTACLE = 100;

    auto& rng = GetRandomGenerator();

    std::uniform_real_distribution<float> sizeDist(
        minSize,
        maxSize
    );

    std::uniform_real_distribution<float> speedDist(
        minSpeed,
        maxSpeed
    );

    std::uniform_real_distribution<float> angleDist(
        0.0f,
        2.0f * PI
    );

    std::uniform_real_distribution<float> probabilityDist(
        0.0f,
        1.0f
    );

    for (int i = 0; i < settings.count; ++i)
    {
        bool generated = false;

        for (int attempt = 0;
             attempt < MAX_ATTEMPTS_PER_OBSTACLE;
             ++attempt)
        {
            Vector3 size = {
                settings.randomSize ? sizeDist(rng) : minSize,
                1.0f,
                settings.randomSize ? sizeDist(rng) : minSize
            };

            size.x = std::min(size.x, terrainWidth);
            size.z = std::min(size.z, terrainDepth);

            const float halfX = size.x * 0.5f;
            const float halfZ = size.z * 0.5f;

            std::uniform_real_distribution<float> xDist(
                bounds.minX + halfX,
                bounds.maxX - halfX
            );

            std::uniform_real_distribution<float> zDist(
                bounds.minZ + halfZ,
                bounds.maxZ - halfZ
            );

            Vector3 position = {
                xDist(rng),
                terrain.groundY + size.y * 0.5f,
                zDist(rng)
            };

            if (!AvoidsProtectedZones(
                position,
                size,
                robotPosition,
                targetPosition,
                protectedRadius))
            {
                continue;
            }

            Obstacle obstacle;

            obstacle.id = AllocateObstacleId();

            if (obstacle.id == 0)
                break;

            obstacle.position = position;
            obstacle.size = size;

            obstacle.isMoving =
                probabilityDist(rng) < movingProbability;

            if (obstacle.isMoving)
            {
                const float angle = angleDist(rng);
                const float speed = speedDist(rng);

                obstacle.velocity = {
                    std::cos(angle) * speed,
                    std::sin(angle) * speed
                };
            }
            else
            {
                obstacle.velocity = { 0.0f, 0.0f };
            }

            obstacles.push_back(obstacle);

            generated = true;
            break;
        }

        if (!generated)
            break;
    }
}


namespace
{
    
    std::pair<float, float> PredictAxisState(
        float position,
        float velocity,
        float min,
        float max,
        float dt
    )
    {
        const float range = max - min;

        if (range <= 0.0f)
        {
            return {
                (min + max) * 0.5f,
                0.0f
            };
        }

        if (!IsValidFloat(position) ||
            !IsValidFloat(velocity) ||
            !IsValidFloat(dt) ||
            dt < 0.0f)
        {
            return {
                IsValidFloat(position)
                    ? std::clamp(position, min, max)
                    : (min + max) * 0.5f,
                0.0f
            };
        }

        const float speed = std::abs(velocity);

        if (speed == 0.0f || dt == 0.0f)
        {
            return {
                std::clamp(position, min, max),
                velocity
            };
        }

        const float period = 2.0f * range;

        if (!IsValidFloat(period) || period <= 0.0f)
            return { position, 0.0f };

        const float clampedPosition =
            std::clamp(position, min, max);

        const float offset = clampedPosition - min;

        float phase = velocity > 0.0f
            ? offset
            : period - offset;

        phase += speed * dt;

        if (!IsValidFloat(phase))
            return { clampedPosition, 0.0f };

        phase = std::fmod(phase, period);

        if (phase < 0.0f)
            phase += period;

        if (phase < range)
        {
            return {
                min + phase,
                speed
            };
        }

        return {
            max - (phase - range),
            -speed
        };
    }

    void UpdateAxis(
        float& position,
        float& velocity,
        float min,
        float max,
        float dt
    )
    {
        if (!IsValidFloat(dt) || dt <= 0.0f)
            return;

        const auto state = PredictAxisState(
            position,
            velocity,
            min,
            max,
            dt
        );

        position = state.first;
        velocity = state.second;
    }
}


void World::Update(float dt)
{
    if (!IsValidFloat(dt) || dt <= 0.0f)
        return;

    const TerrainBounds bounds = GetTerrainBounds(terrain);

    for (Obstacle& obstacle : obstacles)
    {
        if (!obstacle.isMoving)
        {
            obstacle.velocity = { 0.0f, 0.0f };
            continue;
        }

        const float minX =
            bounds.minX + obstacle.size.x * 0.5f;

        const float maxX =
            bounds.maxX - obstacle.size.x * 0.5f;

        const float minZ =
            bounds.minZ + obstacle.size.z * 0.5f;

        const float maxZ =
            bounds.maxZ - obstacle.size.z * 0.5f;

        UpdateAxis(
            obstacle.position.x,
            obstacle.velocity.x,
            minX,
            maxX,
            dt
        );

        UpdateAxis(
            obstacle.position.z,
            obstacle.velocity.y,
            minZ,
            maxZ,
            dt
        );
    }
}


void World::SetTerrain(const TerrainConfig& terrainConfig)
{
    // Valider et appliquer la nouvelle configuration.
    terrain = SanitizeTerrain(terrainConfig);
    ++navigationRevision;

    const TerrainBounds bounds = GetTerrainBounds(terrain);

    for (Obstacle& obstacle : obstacles)
    {
        const float halfX = obstacle.size.x * 0.5f;
        const float halfZ = obstacle.size.z * 0.5f;

        float minX = bounds.minX + halfX;
        float maxX = bounds.maxX - halfX;

        float minZ = bounds.minZ + halfZ;
        float maxZ = bounds.maxZ - halfZ;

        // Obstacle trop large sur X : centrage et immobilisation.
        if (minX > maxX)
        {
            obstacle.position.x = (bounds.minX + bounds.maxX) * 0.5f;
            obstacle.velocity.x = 0.0f;
        }
        else if (obstacle.position.x < minX)
        {
            obstacle.position.x = minX;

            if (obstacle.isMoving)
                obstacle.velocity.x = std::abs(obstacle.velocity.x);
        }
        else if (obstacle.position.x > maxX)
        {
            obstacle.position.x = maxX;

            if (obstacle.isMoving)
                obstacle.velocity.x = -std::abs(obstacle.velocity.x);
        }

        // Obstacle trop large sur Z : centrage et immobilisation.
        if (minZ > maxZ)
        {
            obstacle.position.z = (bounds.minZ + bounds.maxZ) * 0.5f;
            obstacle.velocity.y = 0.0f;
        }
        else if (obstacle.position.z < minZ)
        {
            obstacle.position.z = minZ;

            if (obstacle.isMoving)
                obstacle.velocity.y = std::abs(obstacle.velocity.y);
        }
        else if (obstacle.position.z > maxZ)
        {
            obstacle.position.z = maxZ;

            if (obstacle.isMoving)
                obstacle.velocity.y = -std::abs(obstacle.velocity.y);
        }

        // Repositionner verticalement les obstacles sur le terrain.
        obstacle.position.y =
            terrain.groundY + obstacle.size.y * 0.5f;

        // Garantir que les obstacles statiques restent immobiles.
        if (!obstacle.isMoving)
            obstacle.velocity = { 0.0f, 0.0f };
    }
}


std::uint32_t World::AddObstacle(const Obstacle& obstacle)
{
    // Vérifier les dimensions.
    if (!IsValidFloat(obstacle.size.x) ||
        !IsValidFloat(obstacle.size.y) ||
        !IsValidFloat(obstacle.size.z) ||
        obstacle.size.x <= 0.0f ||
        obstacle.size.y <= 0.0f ||
        obstacle.size.z <= 0.0f)
    {
        return 0;
    }

    // Vérifier les coordonnées horizontales.
    if (!IsValidFloat(obstacle.position.x) ||
        !IsValidFloat(obstacle.position.z))
    {
        return 0;
    }

    // Vérifier la vitesse uniquement pour les obstacles mobiles.
    if (obstacle.isMoving &&
        (!IsValidFloat(obstacle.velocity.x) ||
         !IsValidFloat(obstacle.velocity.y)))
    {
        return 0;
    }

    const TerrainBounds bounds = GetTerrainBounds(terrain);

    const float terrainWidth = bounds.maxX - bounds.minX;
    const float terrainDepth = bounds.maxZ - bounds.minZ;

    if (terrainWidth <= 0.0f || terrainDepth <= 0.0f)
        return 0;

    Obstacle newObstacle = obstacle;

    // L'identifiant fourni par l'appelant est ignoré.
    newObstacle.id = AllocateObstacleId();

    if (newObstacle.id == 0)
        return 0;

    // Les obstacles statiques ne doivent jamais avoir de vitesse.
    if (!newObstacle.isMoving)
        newObstacle.velocity = { 0.0f, 0.0f };

    const float halfX = newObstacle.size.x * 0.5f;
    const float halfZ = newObstacle.size.z * 0.5f;

    // Si l'obstacle est trop grand pour le terrain,
    // on le centre sur l'axe concerné et on annule sa vitesse.
    if (newObstacle.size.x >= terrainWidth)
    {
        newObstacle.position.x =
            (bounds.minX + bounds.maxX) * 0.5f;

        newObstacle.velocity.x = 0.0f;
    }
    else
    {
        newObstacle.position.x = std::clamp(
            newObstacle.position.x,
            bounds.minX + halfX,
            bounds.maxX - halfX
        );
    }

    if (newObstacle.size.z >= terrainDepth)
    {
        newObstacle.position.z =
            (bounds.minZ + bounds.maxZ) * 0.5f;

        newObstacle.velocity.y = 0.0f;
    }
    else
    {
        newObstacle.position.z = std::clamp(
            newObstacle.position.z,
            bounds.minZ + halfZ,
            bounds.maxZ - halfZ
        );
    }

    newObstacle.position.y =
        terrain.groundY + newObstacle.size.y * 0.5f;

    obstacles.push_back(newObstacle);
    ++navigationRevision;

    return newObstacle.id;
}


bool World::RemoveObstacle(std::uint32_t id)
{
    if (id == 0)
        return false;

    const auto it = std::find_if(
        obstacles.begin(),
        obstacles.end(),
        [id](const Obstacle& obstacle)
        {
            return obstacle.id == id;
        }
    );

    if (it == obstacles.end())
        return false;

    obstacles.erase(it);
    ++navigationRevision;

    return true;
}


const std::vector<Obstacle>& World::GetObstacles() const
{
    return obstacles;
}

const TerrainConfig& World::GetTerrain() const
{
    return terrain;
}


Obstacle World::GetPredictedObstacleState(
    const Obstacle& obstacle,
    float futureTime
) const
{
    // Copie de l'état actuel : aucune modification du monde.
    Obstacle predicted = obstacle;

    if (!std::isfinite(futureTime) || futureTime < 0.0f)
        return predicted;

    if (!obstacle.isMoving || futureTime == 0.0f)
        return predicted;

    if (!IsValidFloat(obstacle.size.x) ||
        !IsValidFloat(obstacle.size.z) ||
        obstacle.size.x <= 0.0f ||
        obstacle.size.z <= 0.0f)
    {
        return predicted;
    }

    const float terrainWidth =
        terrain.cellsX * terrain.cellSize;

    const float terrainDepth =
        terrain.cellsZ * terrain.cellSize;

    const float minX = -terrainWidth * 0.5f + obstacle.size.x * 0.5f;
    const float maxX =  terrainWidth * 0.5f - obstacle.size.x * 0.5f;

    const float minZ = -terrainDepth * 0.5f + obstacle.size.z * 0.5f;
    const float maxZ =  terrainDepth * 0.5f - obstacle.size.z * 0.5f;

    // Prédiction indépendante sur les deux axes.
    const auto xState = PredictAxisState(
        obstacle.position.x,
        obstacle.velocity.x,
        minX,
        maxX,
        futureTime
    );

    const auto zState = PredictAxisState(
        obstacle.position.z,
        obstacle.velocity.y,
        minZ,
        maxZ,
        futureTime
    );

    predicted.position.x = xState.first;
    predicted.position.z = zState.first;

    predicted.velocity.x = xState.second;
    predicted.velocity.y = zState.second;

    return predicted;
}

// ============================================
// VERSION DE NAVIGATION
// ============================================

std::uint64_t World::GetNavigationRevision() const
{
    return navigationRevision;
}




/*Il manque trois protections que nous ajouterons :
1. Éviter que deux obstacles soient générés l'un sur l'autre.
2. Éviter de générer un obstacle sur le robot ou sur la cible.
3. Gérer proprement les identifiants lorsque des obstacles apparaissent ou disparaissent pendant la simulation.*/