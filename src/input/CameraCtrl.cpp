
#include "CameraCtrl.h"

#include "raymath.h"

#include <algorithm>
#include <cmath>

void CameraController::Update(
    Camera3D& camera,
    float dt,
    bool uiCapturesKeyboard,
    bool uiCapturesMouse
)
{
    if (!std::isfinite(dt) || dt <= 0.0f)
        return;

    // =========================================
    // DEPLACEMENT DE LA CAMERA
    // =========================================

    if (!uiCapturesKeyboard)
    {
        Vector3 movement = { 0.0f, 0.0f, 0.0f };

        if (IsKeyDown(KEY_W)) movement.z -= 1.0f;
        if (IsKeyDown(KEY_S)) movement.z += 1.0f;
        if (IsKeyDown(KEY_A)) movement.x -= 1.0f;
        if (IsKeyDown(KEY_D)) movement.x += 1.0f;
        if (IsKeyDown(KEY_E)) movement.y += 1.0f;
        if (IsKeyDown(KEY_Q)) movement.y -= 1.0f;

        if (Vector3LengthSqr(movement) > 0.0f)
        {
            movement = Vector3Normalize(movement);
            movement = Vector3Scale(
                movement,
                movementSpeed * dt
            );

            camera.position = Vector3Add(
                camera.position,
                movement
            );

            camera.target = Vector3Add(
                camera.target,
                movement
            );
        }
    }

    // =========================================
    // ZOOM
    // =========================================

    if (!uiCapturesMouse)
    {
        const float wheel = GetMouseWheelMove();

        if (wheel != 0.0f)
        {
            Vector3 direction = Vector3Subtract(
                camera.target,
                camera.position
            );

            float distance = Vector3Length(direction);

            if (distance > 0.0001f)
            {
                direction = Vector3Scale(
                    direction,
                    1.0f / distance
                );

                const float newDistance = std::clamp(
                    distance - wheel * zoomSpeed,
                    minZoomDistance,
                    maxZoomDistance
                );

                camera.position = Vector3Subtract(
                    camera.target,
                    Vector3Scale(direction, newDistance)
                );
            }
        }
    }

    // =========================================
    // ROTATION HORIZONTALE
    // =========================================

    if (!uiCapturesMouse &&
        IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
    {
        const Vector2 mouseDelta = GetMouseDelta();

        Vector3 offset = Vector3Subtract(
            camera.position,
            camera.target
        );

        const float angle =
            -mouseDelta.x * rotationSensitivity;

        offset = Vector3RotateByAxisAngle(
            offset,
            { 0.0f, 1.0f, 0.0f },
            angle
        );

        camera.position = Vector3Add(
            camera.target,
            offset
        );
    }
}
