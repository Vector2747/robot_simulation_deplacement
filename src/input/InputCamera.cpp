// Commandes de camera existantes.
        float cameraSpeed = 0.25f;

        if (IsKeyDown(KEY_W)) camera.position.z -= cameraSpeed;
        if (IsKeyDown(KEY_S)) camera.position.z += cameraSpeed;
        if (IsKeyDown(KEY_A)) camera.position.x -= cameraSpeed;
        if (IsKeyDown(KEY_D)) camera.position.x += cameraSpeed;
        if (IsKeyDown(KEY_E)) camera.position.y += cameraSpeed;
        if (IsKeyDown(KEY_Q)) camera.position.y -= cameraSpeed;

        float wheel = GetMouseWheelMove();

        if (wheel != 0.0f) {
            Vector3 direction =
                Vector3Subtract(camera.target, camera.position);

            direction = Vector3Scale(
                Vector3Normalize(direction), wheel * 1.5f
            );

            camera.position = Vector3Add(camera.position, direction);
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 mouseDelta = GetMouseDelta();

            Vector3 offset =
                Vector3Subtract(camera.position, camera.target);

            float angle = -mouseDelta.x * 0.005f;

            offset = Vector3RotateByAxisAngle(
                offset, {0.0f, 1.0f, 0.0f}, angle
            );

            camera.position = Vector3Add(camera.target, offset);
        }

        // Bouton AUTO ON/OFF
        Rectangle autoButton = {20, 110, 190, 42};
        Vector2 mouse = GetMousePosition();

        bool buttonHovered = CheckCollisionPointRec(mouse, autoButton);