
#include "Robot.h"

#include <algorithm>
#include <cmath>

Robot::Robot(
    const RobotConfig& config,
    Vector3 initialPosition
)
    : position(initialPosition),
      velocity{ 0.0f, 0.0f },
      radius(std::max(0.0f, config.radius)),
      maxSpeed(std::max(0.0f, config.speed))
{
}

void Robot::SetVelocity(Vector2 newVelocity)
{
    if (!std::isfinite(newVelocity.x) ||
        !std::isfinite(newVelocity.y))
    {
        Stop();
        return;
    }

    const float speedSquared =
        newVelocity.x * newVelocity.x +
        newVelocity.y * newVelocity.y;

    const float maxSpeedSquared = maxSpeed * maxSpeed;

    if (speedSquared > maxSpeedSquared && speedSquared > 0.0f)
    {
        const float scale = maxSpeed / std::sqrt(speedSquared);

        newVelocity.x *= scale;
        newVelocity.y *= scale;
    }

    velocity = newVelocity;
}

void Robot::SetRadius(float newRadius)
{
    if (!std::isfinite(newRadius))
        return;

    radius = std::max(0.0f, newRadius);
}

void Robot::SetMaxSpeed(float newMaxSpeed)
{
    if (!std::isfinite(newMaxSpeed))
        return;

    maxSpeed = std::max(0.0f, newMaxSpeed);

    // Adapter immédiatement la vitesse actuelle à la nouvelle limite.
    SetVelocity(velocity);
}

void Robot::Update(float dt)
{
    if (!std::isfinite(dt) || dt <= 0.0f)
        return;

    position.x += velocity.x * dt;
    position.z += velocity.y * dt;
}

void Robot::Stop()
{
    velocity = { 0.0f, 0.0f };
}

Vector3 Robot::GetPosition() const
{
    return position;
}

Vector2 Robot::GetVelocity() const
{
    return velocity;
}

float Robot::GetRadius() const
{
    return radius;
}

float Robot::GetMaxSpeed() const
{
    return maxSpeed;
}
