
#pragma once

#include "raylib.h"

class CameraController
{
private:
    float movementSpeed = 15.0f;
    float zoomSpeed = 1.5f;

    float minZoomDistance = 5.0f;
    float maxZoomDistance = 150.0f;

    float rotationSensitivity = 0.005f;

public:
    void Update(
        Camera3D& camera,
        float dt,
        bool uiCapturesKeyboard,
        bool uiCapturesMouse
    );
};
