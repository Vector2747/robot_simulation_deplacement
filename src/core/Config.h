#pragma once

struct TerrainConfig
{
    int cellsX = 400;
    int cellsZ = 400;

    float cellSize = 1.0f;

    float groundY = 0.0f;

    bool showGrid = true;
};

struct RobotConfig
{
    float radius = 0.65f;
    float speed = 3.0f;
};

struct ObstacleGenerationConfig
{
    int count = 25;

    float minSize = 1.0f;
    float maxSize = 4.0f;

    float minSpeed = 0.5f;
    float maxSpeed = 2.5f;

    float movingProbability = 0.5f;

    bool randomSize = true;
};

struct NavigationConfig
{
    float arrivalRadius = 1.0f;
};

struct SimulationConfig
{
    TerrainConfig terrain;
    RobotConfig robot;
    ObstacleGenerationConfig obstacleGeneration;
    NavigationConfig navigation;
};

