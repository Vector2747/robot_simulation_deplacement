// Deplacement manuel fluide du robot ou de la cible
        bool canMoveManually = !autoMode || controlTarget;

        if (canMoveManually ){//&& !uiCapturesKeyboard) {
            Vector3& controlledObject = controlTarget ? target : robot;

            const float manualSpeed = 4.0f; // Unites par seconde
            //float dt = GetFrameTime();

            float dx = 0.0f;
            float dz = 0.0f;

            if (IsKeyDown(KEY_UP))    dz -= 1.0f;
            if (IsKeyDown(KEY_DOWN))  dz += 1.0f;
            if (IsKeyDown(KEY_LEFT))  dx -= 1.0f;
            if (IsKeyDown(KEY_RIGHT)) dx += 1.0f;

            // Normaliser la direction pour conserver la meme vitesse en diagonale
            float length = std::sqrt(dx * dx + dz * dz);

            bool positionChanged = length > 0.0f;

            if (positionChanged) {
                dx /= length;
                dz /= length;

                controlledObject.x += dx * manualSpeed * dt;
                controlledObject.z += dz * manualSpeed * dt;

                if (controlTarget) {
                    targetVelocity.x = dx * manualSpeed;
                    targetVelocity.y = dz * manualSpeed;
                }

                // Limites de la carte
                controlledObject.x = Clamp(
                    controlledObject.x,
                    terrainMinX + config.robot.radius,
                    terrainMaxX - config.robot.radius
                );

                controlledObject.z = Clamp(
                    controlledObject.z,
                    terrainMinZ + config.robot.radius,
                    terrainMaxZ - config.robot.radius
                );

                
                
            }

            if (controlTarget && length == 0.0f) {
                targetVelocity = {0.0f, 0.0f};
            }
        }