#pragma once

#include "raylib.h"

#include <cstdint>

struct Obstacle
{
    std::uint32_t id = 0;

    Vector3 position = { 0.0f, 0.0f, 0.0f };
    Vector3 size = { 1.0f, 1.0f, 1.0f };

    Vector2 velocity = { 0.0f, 0.0f };

    bool isMoving = false;
};