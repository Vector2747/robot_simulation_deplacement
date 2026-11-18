#pragma once

#include "raylib.h"
#include "../core/Config.h"

class Robot
{
private:
    Vector3 position;
    Vector2 velocity;

    float radius;
    float maxSpeed;

public:
    explicit Robot(
        const RobotConfig& config,
        Vector3 initialPosition
    );

    void SetVelocity(Vector2 newVelocity);
    void Update(float dt);

    void Stop();

    Vector3 GetPosition() const;
    Vector2 GetVelocity() const;

    float GetRadius() const;
    float GetMaxSpeed() const;
    void SetRadius(float newRadius);
    void SetMaxSpeed(float newMaxSpeed);
};