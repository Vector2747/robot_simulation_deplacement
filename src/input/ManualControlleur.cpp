#include "ManualControlleur.h"
#include "raylib.h"

#include <cmath>

Vector2 ManualController::CalculateInputVelocity(
    float inputX,
    float inputZ,
    float speed
)
{
    if (!std::isfinite(inputX) ||
        !std::isfinite(inputZ) ||
        !std::isfinite(speed) ||
        speed <= 0.0f)
    {
        return { 0.0f, 0.0f };
    }

    const float inputLength = std::sqrt(
        inputX * inputX + inputZ * inputZ
    );

    if (inputLength <= 0.0f)
        return { 0.0f, 0.0f };

    const float scaledSpeed = speed / inputLength;
    return { inputX * scaledSpeed, inputZ * scaledSpeed };
}

void ManualController::Update(
    Robot& robot,
    Target& target,
    float dt,
    bool uiCapturesKeyboard
)
{
    if (uiCapturesKeyboard || !std::isfinite(dt) || dt <= 0.0f)
    {
        robot.Stop();
        target.Stop();
        return;
    }

    // ============================================
    // CHANGEMENT DE MODE
    // ============================================

    if (IsKeyPressed(KEY_T))
    {
        controlTarget = !controlTarget;

        if (controlTarget)
        {
            if (!automaticNavigation)
                robot.Stop();
        }
        else
        {
            target.Stop();
            if (!automaticNavigation)
                robot.Stop();
        }
    }

    // ============================================
    // ACTIVATION NAVIGATION AUTOMATIQUE
    // ============================================

    if (IsKeyPressed(KEY_N))
    {
        automaticNavigation = !automaticNavigation;

        if (!automaticNavigation)
        {
            robot.Stop();
            target.Stop();
        }
    }

    // ============================================
    // CONTROLE MANUEL DE LA CIBLE
    // ============================================

    if (controlTarget)
    {
        float dx = 0.0f;
        float dz = 0.0f;

        if (IsKeyDown(KEY_UP))    dz -= 1.0f;
        if (IsKeyDown(KEY_DOWN))  dz += 1.0f;
        if (IsKeyDown(KEY_LEFT))  dx -= 1.0f;
        if (IsKeyDown(KEY_RIGHT)) dx += 1.0f;

        if (!automaticNavigation)
            robot.Stop();

        target.SetVelocity(
            CalculateInputVelocity(dx, dz, targetSpeed)
        );
        return;
    }

    // ============================================
    // MODE AUTOMATIQUE
    // ============================================

    if (automaticNavigation)
    {
        target.Stop();
        return;
    }

    // ============================================
    // CONTROLE MANUEL
    // ============================================

    float dx = 0.0f;
    float dz = 0.0f;

    if (IsKeyDown(KEY_UP))    dz -= 1.0f;
    if (IsKeyDown(KEY_DOWN))  dz += 1.0f;
    if (IsKeyDown(KEY_LEFT))  dx -= 1.0f;
    if (IsKeyDown(KEY_RIGHT)) dx += 1.0f;

    const Vector2 velocity =
        CalculateInputVelocity(dx, dz, robot.GetMaxSpeed());

    target.Stop();
    robot.SetVelocity(velocity);
}


// ============================================
// ACCESSEURS
// ============================================

bool ManualController::IsControllingTarget() const
{
    return controlTarget;
}


bool ManualController::IsAutomaticNavigation() const
{
    return automaticNavigation;
}