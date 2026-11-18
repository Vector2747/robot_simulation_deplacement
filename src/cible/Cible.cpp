
#include "Cible.h"

#include <cmath>

Target::Target(Vector3 initialPosition)
    : position(initialPosition),
      velocity{ 0.0f, 0.0f }
{
}

void Target::SetPosition(Vector3 newPosition)
{
    position = newPosition;
}

void Target::SetVelocity(Vector2 newVelocity)
{
    if (!std::isfinite(newVelocity.x) ||
        !std::isfinite(newVelocity.y))
    {
        Stop();
        return;
    }

    velocity = newVelocity;
}

void Target::Update(float dt)
{
    if (!std::isfinite(dt) || dt <= 0.0f)
        return;

    position.x += velocity.x * dt;
    position.z += velocity.y * dt;
}

void Target::Stop()
{
    velocity = { 0.0f, 0.0f };
}

Vector3 Target::GetPosition() const
{
    return position;
}

Vector2 Target::GetVelocity() const
{
    return velocity;
}
