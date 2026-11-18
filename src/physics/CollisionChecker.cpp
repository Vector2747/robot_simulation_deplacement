
#include "CollisionChecker.h"

#include <algorithm>
#include <cmath>

namespace
{
    bool IsValidFloat(float value)
    {
        return std::isfinite(value);
    }

    bool IsValidCircle(Vector3 position, float radius)
    {
        return IsValidFloat(position.x) &&
               IsValidFloat(position.z) &&
               IsValidFloat(radius) &&
               radius >= 0.0f;
    }

    bool IsValidObstacle(const Obstacle& obstacle)
    {
        return IsValidFloat(obstacle.position.x) &&
               IsValidFloat(obstacle.position.z) &&
               IsValidFloat(obstacle.size.x) &&
               IsValidFloat(obstacle.size.z) &&
               obstacle.size.x > 0.0f &&
               obstacle.size.z > 0.0f;
    }

    bool IsValidTerrain(const TerrainConfig& terrain)
    {
        if (terrain.cellsX <= 0 || terrain.cellsZ <= 0)
            return false;

        if (!IsValidFloat(terrain.cellSize) ||
            terrain.cellSize <= 0.0f)
            return false;

        const float widthX = terrain.cellsX * terrain.cellSize;
        const float widthZ = terrain.cellsZ * terrain.cellSize;

        return IsValidFloat(widthX) &&
               IsValidFloat(widthZ) &&
               widthX > 0.0f &&
               widthZ > 0.0f;
    }
}

bool CollisionChecker::CircleIntersectsObstacle(
    Vector3 center,
    float radius,
    const Obstacle& obstacle
)
{
    if (!IsValidCircle(center, radius) ||
        !IsValidObstacle(obstacle))
    {
        return true;
    }

    const float halfWidth = obstacle.size.x * 0.5f;
    const float halfDepth = obstacle.size.z * 0.5f;

    const float minX = obstacle.position.x - halfWidth;
    const float maxX = obstacle.position.x + halfWidth;

    const float minZ = obstacle.position.z - halfDepth;
    const float maxZ = obstacle.position.z + halfDepth;

    if (!IsValidFloat(minX) || !IsValidFloat(maxX) ||
        !IsValidFloat(minZ) || !IsValidFloat(maxZ))
    {
        return true;
    }

    const float closestX = std::clamp(center.x, minX, maxX);
    const float closestZ = std::clamp(center.z, minZ, maxZ);

    const float dx = center.x - closestX;
    const float dz = center.z - closestZ;

    return dx * dx + dz * dz <= radius * radius;
}

bool CollisionChecker::IsPositionFree(
    Vector3 position,
    float radius,
    const std::vector<Obstacle>& obstacles
)
{
    if (!IsValidCircle(position, radius))
        return false;

    for (const Obstacle& obstacle : obstacles)
    {
        if (CircleIntersectsObstacle(position, radius, obstacle))
            return false;
    }

    return true;
}

bool CollisionChecker::IsInsideTerrain(
    Vector3 position,
    float radius,
    const TerrainConfig& terrain
)
{
    if (!IsValidCircle(position, radius) ||
        !IsValidTerrain(terrain))
    {
        return false;
    }

    const float halfWidth =
        terrain.cellsX * terrain.cellSize * 0.5f;

    const float halfDepth =
        terrain.cellsZ * terrain.cellSize * 0.5f;

    const float minX = -halfWidth;
    const float maxX = halfWidth;

    const float minZ = -halfDepth;
    const float maxZ = halfDepth;

    return position.x - radius > minX &&
           position.x + radius < maxX &&
           position.z - radius > minZ &&
           position.z + radius < maxZ;
}

bool CollisionChecker::IsPositionSafe(
    Vector3 position,
    float radius,
    const TerrainConfig& terrain,
    const std::vector<Obstacle>& obstacles
)
{
    return IsInsideTerrain(position, radius, terrain) &&
           IsPositionFree(position, radius, obstacles);
}



float PointToSegmentDistanceSquared(
    float px, float pz,
    float ax, float az,
    float bx, float bz
)
{
    const float dx = bx - ax;
    const float dz = bz - az;

    const float lengthSquared = dx * dx + dz * dz;

    if (lengthSquared <= 0.0f)
    {
        const float ex = px - ax;
        const float ez = pz - az;
        return ex * ex + ez * ez;
    }

    const float t = std::clamp(
        ((px - ax) * dx + (pz - az) * dz) / lengthSquared,
        0.0f,
        1.0f
    );

    const float closestX = ax + t * dx;
    const float closestZ = az + t * dz;

    const float ex = px - closestX;
    const float ez = pz - closestZ;

    return ex * ex + ez * ez;
}

float PointToRectangleDistanceSquared(
    float px, float pz,
    float minX, float maxX,
    float minZ, float maxZ
)
{
    const float dx = std::max(
        { minX - px, 0.0f, px - maxX }
    );

    const float dz = std::max(
        { minZ - pz, 0.0f, pz - maxZ }
    );

    return dx * dx + dz * dz;
}

bool SegmentIntersectsRectangle(
    float ax, float az,
    float bx, float bz,
    float minX, float maxX,
    float minZ, float maxZ
)
{
    float tMin = 0.0f;
    float tMax = 1.0f;

    const float dx = bx - ax;
    const float dz = bz - az;

    const auto Clip = [&](float p, float q) -> bool
    {
        if (p == 0.0f)
            return q >= 0.0f;

        const float t = q / p;

        if (p < 0.0f)
            tMin = std::max(tMin, t);
        else
            tMax = std::min(tMax, t);

        return tMin <= tMax;
    };

    return Clip(-dx, ax - minX) &&
           Clip( dx, maxX - ax) &&
           Clip(-dz, az - minZ) &&
           Clip( dz, maxZ - az);
}


bool SegmentsIntersect(
    float ax, float az,
    float bx, float bz,
    float cx, float cz,
    float dx, float dz
)
{
    const auto Cross = [](
        float ax, float az,
        float bx, float bz,
        float cx, float cz)
    {
        return (bx - ax) * (cz - az) -
               (bz - az) * (cx - ax);
    };

    const float o1 = Cross(ax, az, bx, bz, cx, cz);
    const float o2 = Cross(ax, az, bx, bz, dx, dz);
    const float o3 = Cross(cx, cz, dx, dz, ax, az);
    const float o4 = Cross(cx, cz, dx, dz, bx, bz);

    constexpr float epsilon = 1e-6f;

    if (((o1 > epsilon && o2 < -epsilon) ||
         (o1 < -epsilon && o2 > epsilon)) &&
        ((o3 > epsilon && o4 < -epsilon) ||
         (o3 < -epsilon && o4 > epsilon)))
        return true;

    const auto OnSegment = [&](float px, float pz,
                               float qx, float qz,
                               float rx, float rz)
    {
        return std::abs(Cross(px, pz, qx, qz, rx, rz)) <= epsilon &&
               rx >= std::min(px, qx) - epsilon &&
               rx <= std::max(px, qx) + epsilon &&
               rz >= std::min(pz, qz) - epsilon &&
               rz <= std::max(pz, qz) + epsilon;
    };

    return OnSegment(ax, az, bx, bz, cx, cz) ||
           OnSegment(ax, az, bx, bz, dx, dz) ||
           OnSegment(cx, cz, dx, dz, ax, az) ||
           OnSegment(cx, cz, dx, dz, bx, bz);
}

float SegmentToSegmentDistanceSquared(
    float ax, float az,
    float bx, float bz,
    float cx, float cz,
    float dx, float dz
)
{
    if (SegmentsIntersect(
        ax, az, bx, bz, cx, cz, dx, dz))
        return 0.0f;

    return std::min({
        PointToSegmentDistanceSquared(ax, az, cx, cz, dx, dz),
        PointToSegmentDistanceSquared(bx, bz, cx, cz, dx, dz),
        PointToSegmentDistanceSquared(cx, cz, ax, az, bx, bz),
        PointToSegmentDistanceSquared(dx, dz, ax, az, bx, bz)
    });
}

bool SegmentIntersectsCircleExpandedRectangle(
    float ax, float az,
    float bx, float bz,
    float radius,
    const Obstacle& obstacle
)
{
    const float halfWidth = obstacle.size.x * 0.5f;
    const float halfDepth = obstacle.size.z * 0.5f;

    const float minX = obstacle.position.x - halfWidth;
    const float maxX = obstacle.position.x + halfWidth;
    const float minZ = obstacle.position.z - halfDepth;
    const float maxZ = obstacle.position.z + halfDepth;

    if (!IsValidFloat(minX) || !IsValidFloat(maxX) ||
        !IsValidFloat(minZ) || !IsValidFloat(maxZ))
        return true;

    if (SegmentIntersectsRectangle(
        ax, az, bx, bz, minX, maxX, minZ, maxZ))
        return true;

    const float radiusSquared = radius * radius;

    const float corners[4][2] =
    {
        { minX, minZ },
        { minX, maxZ },
        { maxX, maxZ },
        { maxX, minZ }
    };

    for (int i = 0; i < 4; ++i)
    {
        const int j = (i + 1) % 4;

        if (SegmentToSegmentDistanceSquared(
            ax, az, bx, bz,
            corners[i][0], corners[i][1],
            corners[j][0], corners[j][1])
            <= radiusSquared)
            return true;
    }

    return false;
}



bool CollisionChecker::IsMovementSafe(
    Vector3 start,
    Vector3 end,
    float radius,
    const TerrainConfig& terrain,
    const std::vector<Obstacle>& obstacles
)
{
    if (!IsValidCircle(start, radius) ||
        !IsValidCircle(end, radius))
        return false;

    // Le terrain est rectangulaire et convexe :
    // si les deux extrémités sont à l'intérieur,
    // tout le segment reste à l'intérieur.
    if (!IsInsideTerrain(start, radius, terrain) ||
        !IsInsideTerrain(end, radius, terrain))
        return false;

    for (const Obstacle& obstacle : obstacles)
    {
        if (!IsValidObstacle(obstacle))
            return false;

        if (SegmentIntersectsCircleExpandedRectangle(
            start.x,
            start.z,
            end.x,
            end.z,
            radius,
            obstacle))
        {
            return false;
        }
    }

    return true;
}
