Vector3 PredictObstaclePosition(
    const Obstacle& obstacle,
    float futureTime,
    const TerrainConfig& terrain
) {
    Vector3 futurePosition = obstacle.position;

    futurePosition.x += obstacle.velocity.x * futureTime;
    futurePosition.z += obstacle.velocity.y * futureTime;

    const float minX =
        terrain.originX + obstacle.size.x * 0.5f;

    const float maxX =
        terrain.originX +
        terrain.cellsX * terrain.cellSize -
        obstacle.size.x * 0.5f;

    const float minZ =
        terrain.originZ + obstacle.size.z * 0.5f;

    const float maxZ =
        terrain.originZ +
        terrain.cellsZ * terrain.cellSize -
        obstacle.size.z * 0.5f;

    auto reflectPosition = [](float position,
                              float min,
                              float max) {
        const float range = max - min;

        if (range <= 0.0f) return min;

        float offset = std::fmod(position - min, 2.0f * range);

        if (offset < 0.0f) offset += 2.0f * range;

        return offset <= range
            ? min + offset
            : max - (offset - range);
    };

    futurePosition.x =
        reflectPosition(futurePosition.x, minX, maxX);

    futurePosition.z =
        reflectPosition(futurePosition.z, minZ, maxZ);

    return futurePosition;
}