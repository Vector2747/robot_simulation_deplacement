#pragma once

#include "raylib.h"
#include "../robot/Robot.h"
#include "../cible/Cible.h"

class ManualController
{
private:
    bool controlTarget = false;
    bool automaticNavigation = false;

    float targetSpeed = 4.0f;

public:
    static Vector2 CalculateInputVelocity(
        float inputX,
        float inputZ,
        float speed
    );

    void Update(
        Robot& robot,
        Target& target,
        float dt,
        bool uiCapturesKeyboard
    );

    bool IsControllingTarget() const;

    bool IsAutomaticNavigation() const;
};