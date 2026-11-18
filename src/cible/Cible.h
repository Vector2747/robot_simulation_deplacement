
#pragma once

#include "raylib.h"

class Target
{
private:
    Vector3 position;
    Vector2 velocity;

public:
    explicit Target(Vector3 initialPosition);

    void SetPosition(Vector3 newPosition);
    void SetVelocity(Vector2 newVelocity);

    void Update(float dt);
    void Stop();

    Vector3 GetPosition() const;
    Vector2 GetVelocity() const;
};
