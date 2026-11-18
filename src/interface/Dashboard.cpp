
#include "Dashboard.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <string>

void DrawDashboard(
    World& world,
    Robot& robot,
    const Target& target,
    const ManualController& manualController,
    NavigationController& navigationController,
    EvilAhrEvent& evilAhrEvent,
    SimulationConfig& config
)
{
    // ============================================
    // CONFIGURATION DE LA FENETRE
    // ============================================

    ImGui::SetNextWindowPos(
        ImVec2(15.0f, 15.0f),
        ImGuiCond_FirstUseEver
    );

    ImGui::SetNextWindowSize(
        ImVec2(400.0f, 760.0f),
        ImGuiCond_FirstUseEver
    );

    if (ImGui::Begin("ROBOT NAVIGATION"))
    {
        // ========================================
        // MODE DE CONTROLE
        // ========================================

        ImGui::Text("CONTROLE");
        ImGui::Separator();

        const bool controllingTarget =
            manualController.IsControllingTarget();
        const bool automaticNavigation =
            manualController.IsAutomaticNavigation();

        if (controllingTarget)
        {
            ImGui::TextColored(
                ImVec4(1.0f, 0.35f, 0.35f, 1.0f),
                automaticNavigation
                    ? "Mode actif : CIBLE + NAVIGATION AUTO"
                    : "Mode actif : CIBLE"
            );
        }
        else
        {
            ImGui::TextColored(
                ImVec4(0.25f, 0.65f, 1.0f, 1.0f),
                "Mode actif : ROBOT"
            );
        }

        ImGui::Text("T : choisir le robot ou la cible");
        ImGui::Text("N : activer/desactiver la navigation auto");

        // ========================================
        // PARAMETRES DU TERRAIN
        // ========================================

        ImGui::Spacing();
        ImGui::Separator();

        if (ImGui::CollapsingHeader(
            "PARAMETRES DU TERRAIN",
            ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::SliderInt(
                "Cellules X",
                &config.terrain.cellsX,
                1,
                1000
            );

            ImGui::SliderInt(
                "Cellules Z",
                &config.terrain.cellsZ,
                1,
                1000
            );

            ImGui::SliderFloat(
                "Taille cellule",
                &config.terrain.cellSize,
                0.1f,
                5.0f,
                "%.2f"
            );

            ImGui::SliderFloat(
                "Hauteur du sol",
                &config.terrain.groundY,
                -10.0f,
                10.0f,
                "%.2f"
            );

            ImGui::Checkbox(
                "Afficher la grille",
                &config.terrain.showGrid
            );

            if (ImGui::Button("Appliquer le terrain"))
            {
                world.SetTerrain(config.terrain);
            }

            const TerrainConfig& terrain =
                world.GetTerrain();

            ImGui::Text(
                "Terrain actuel : %.1f x %.1f",
                terrain.cellsX * terrain.cellSize,
                terrain.cellsZ * terrain.cellSize
            );
        }

        // ========================================
        // GENERATION DES OBSTACLES
        // ========================================

        ImGui::Spacing();
        ImGui::Separator();

        if (ImGui::CollapsingHeader(
            "GENERATION DES OBSTACLES",
            ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::SliderInt(
                "Nombre",
                &config.obstacleGeneration.count,
                0,
                200
            );

            ImGui::SliderFloat(
                "Taille minimale",
                &config.obstacleGeneration.minSize,
                0.1f,
                20.0f,
                "%.2f"
            );

            ImGui::SliderFloat(
                "Taille maximale",
                &config.obstacleGeneration.maxSize,
                0.1f,
                20.0f,
                "%.2f"
            );

            ImGui::Checkbox(
                "Tailles aleatoires",
                &config.obstacleGeneration.randomSize
            );

            ImGui::Spacing();

            ImGui::SliderFloat(
                "Vitesse minimale",
                &config.obstacleGeneration.minSpeed,
                0.0f,
                10.0f,
                "%.2f"
            );

            ImGui::SliderFloat(
                "Vitesse maximale",
                &config.obstacleGeneration.maxSpeed,
                0.0f,
                10.0f,
                "%.2f"
            );

            ImGui::SliderFloat(
                "Proportion mobile",
                &config.obstacleGeneration.movingProbability,
                0.0f,
                1.0f,
                "%.2f"
            );

            if (ImGui::Button("Generer les obstacles"))
            {
                // Garantir la coherence des intervalles.
                config.obstacleGeneration.maxSize =
                    std::max(
                        config.obstacleGeneration.minSize,
                        config.obstacleGeneration.maxSize
                    );

                config.obstacleGeneration.maxSpeed =
                    std::max(
                        config.obstacleGeneration.minSpeed,
                        config.obstacleGeneration.maxSpeed
                    );

                world.GenerateObstacles(
                    config.obstacleGeneration,
                    config.robot,
                    robot.GetPosition(),
                    target.GetPosition()
                );
            }

            const auto& obstacles =
                world.GetObstacles();

            int movingObstacleCount = 0;

            for (const Obstacle& obstacle : obstacles)
            {
                if (obstacle.isMoving)
                    ++movingObstacleCount;
            }

            ImGui::Text(
                "Obstacles generes : %d / %d",
                static_cast<int>(obstacles.size()),
                config.obstacleGeneration.count
            );

            ImGui::Text(
                "Obstacles mobiles : %d",
                movingObstacleCount
            );
        }

        ImGui::Spacing();
        ImGui::Separator();

        if (ImGui::CollapsingHeader(
            "EVENEMENT EVIL AHR",
            ImGuiTreeNodeFlags_DefaultOpen))
        {
            EvilAhrConfig& evilConfig = evilAhrEvent.GetConfig();
            const EvilAhrDiagnostics& evilDiagnostics =
                evilAhrEvent.GetDiagnostics();

            ImGui::Checkbox("Activer Evil Ahr", &evilConfig.enabled);
            if (ImGui::Button("Declencher une vague"))
                evilAhrEvent.RequestWave();
            ImGui::SameLine();
            if (ImGui::Button("Reinitialiser Evil Ahr"))
                evilAhrEvent.Reset(world);

            ImGui::SliderFloat(
                "Niveau d'emmerdement",
                &evilConfig.annoyancePercent,
                0.0f,
                100.0f,
                "%.0f%%"
            );
            ImGui::SliderFloat(
                "Frequence d'apparition (s)",
                &evilConfig.spawnFrequencySeconds,
                0.25f,
                30.0f,
                "%.2f"
            );
            ImGui::SliderInt(
                "Maximum d'obstacles actifs",
                &evilConfig.maximumActiveObstacles,
                1,
                200
            );
            ImGui::SliderFloat(
                "Proportion dynamique",
                &evilConfig.movingObstaclePercent,
                0.0f,
                100.0f,
                "%.0f%%"
            );
            ImGui::SliderFloat(
                "Duree de vie (s)",
                &evilConfig.lifetimeSeconds,
                0.5f,
                60.0f,
                "%.1f"
            );

            ImGui::Text(
                "Obstacles monde : %d / %d | Evil Ahr actifs : %d",
                static_cast<int>(world.GetObstacles().size()),
                evilConfig.maximumActiveObstacles,
                evilDiagnostics.activeOwnedObstacles
            );
            ImGui::Text(
                "Vagues : %llu | apparus : %llu | disparus : %llu",
                static_cast<unsigned long long>(
                    evilDiagnostics.wavesTriggered),
                static_cast<unsigned long long>(
                    evilDiagnostics.obstaclesSpawned),
                static_cast<unsigned long long>(
                    evilDiagnostics.obstaclesRemoved)
            );
            ImGui::Text(
                "Derniere vague : %d obstacle(s) | %s",
                evilDiagnostics.lastWaveSpawned,
                evilDiagnostics.lastEvent.c_str()
            );
        }

        // ========================================
        // PARAMETRES DU ROBOT
        // ========================================

        ImGui::Spacing();
        ImGui::Separator();

        if (ImGui::CollapsingHeader(
            "PARAMETRES DU ROBOT",
            ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::SliderFloat(
                "Rayon",
                &config.robot.radius,
                0.1f,
                5.0f,
                "%.2f"
            );

            if (ImGui::SliderFloat(
                "Vitesse maximale",
                &config.robot.speed,
                0.0f,
                20.0f,
                "%.2f"
            ))
            {
                robot.SetMaxSpeed(config.robot.speed);
            }

            if (ImGui::Button("Appliquer les parametres du robot"))
            {
                robot.SetRadius(config.robot.radius);
                robot.SetMaxSpeed(config.robot.speed);
            }

            ImGui::Text(
                "Rayon actuel : %.2f",
                robot.GetRadius()
            );

            ImGui::Text(
                "Limite de vitesse active : %.2f m/s",
                robot.GetMaxSpeed()
            );
        }

        // ========================================
        // INFORMATIONS SUR LE ROBOT
        // ========================================

        ImGui::Spacing();
        ImGui::Separator();

        if (ImGui::CollapsingHeader(
            "ETAT DU ROBOT",
            ImGuiTreeNodeFlags_DefaultOpen))
        {
            const Vector3 robotPosition =
                robot.GetPosition();

            const Vector2 robotVelocity =
                robot.GetVelocity();
            const float appliedSpeed = std::sqrt(
                robotVelocity.x * robotVelocity.x +
                robotVelocity.y * robotVelocity.y
            );

            ImGui::Text(
                "Position : X %.2f | Y %.2f | Z %.2f",
                robotPosition.x,
                robotPosition.y,
                robotPosition.z
            );

            ImGui::Text(
                "Vitesse appliquee : %.2f m/s (X %.2f | Z %.2f)",
                appliedSpeed,
                robotVelocity.x,
                robotVelocity.y
            );
            const float speedLimit = robot.GetMaxSpeed();
            const float speedRatio = speedLimit > 0.0f
                ? std::clamp(appliedSpeed / speedLimit, 0.0f, 1.0f)
                : 0.0f;
            ImGui::ProgressBar(
                speedRatio,
                ImVec2(-1.0f, 0.0f),
                speedLimit > 0.0f
                    ? "Vitesse reelle / limite"
                    : "Vitesse maximale nulle"
            );
        }

        // ========================================
        // INFORMATIONS SUR LA CIBLE
        // ========================================

        ImGui::Spacing();
        ImGui::Separator();

        if (ImGui::CollapsingHeader("ETAT DE LA CIBLE"))
        {
            const Vector3 targetPosition =
                target.GetPosition();

            const Vector2 targetVelocity =
                target.GetVelocity();

            ImGui::Text(
                "Position : X %.2f | Y %.2f | Z %.2f",
                targetPosition.x,
                targetPosition.y,
                targetPosition.z
            );

            ImGui::Text(
                "Vitesse : X %.2f | Z %.2f",
                targetVelocity.x,
                targetVelocity.y
            );
        }

        
        // ========================================
        // NAVIGATION
        // ========================================

        ImGui::Spacing();
        ImGui::Separator();

        if (ImGui::CollapsingHeader(
            "NAVIGATION",
            ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::SliderFloat(
                "Rayon d'arrivee",
                &config.navigation.arrivalRadius,
                0.1f,
                10.0f,
                "%.2f m"
            );

            ImGui::Spacing();

            ImGui::Text(
                "Distance a la cible : %.2f m",
                navigationController.GetDistanceToTarget(
                    robot,
                    target
                )
            );

            ImGui::Text("Etat :");

            switch (navigationController.GetState())
            {
            case NavigationState::Idle:
                ImGui::TextColored(
                    ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                    "IDLE"
                );
                break;

            case NavigationState::Moving:
                ImGui::TextColored(
                    ImVec4(0.25f, 0.65f, 1.0f, 1.0f),
                    "MOVING"
                );
                break;

            case NavigationState::TargetReached:
                ImGui::TextColored(
                    ImVec4(0.2f, 0.9f, 0.35f, 1.0f),
                    "TARGET REACHED"
                );
                break;
            }
        }
        bool stableMovement =
            navigationController.IsStableWindowMovement();

        if (ImGui::Checkbox(
            "Deplacement stable de la grille locale",
            &stableMovement))
        {
            navigationController.SetStableWindowMovement(
                stableMovement
            );
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("TEMPORAL A*");

        if (navigationController.IsTemporalPathActive())
            ImGui::TextColored(
                ImVec4(0.2f, 1.0f, 0.2f, 1.0f),
                "Etat : ACTIVE"
            );
        else
            ImGui::Text("Etat : INACTIVE");

        if (!navigationController.HasTemporalSearchRun())
        {
            ImGui::Text("Derniere recherche : NON EXECUTEE");
        }
        else
        {
            ImGui::Text(
                "Derniere recherche : %s",
                navigationController.WasLastTemporalSearchSuccessful()
                    ? "SUCCES"
                    : "ECHEC"
            );
        }

        ImGui::Text(
            "Noeuds explores : %d",
            navigationController.GetTemporalExpandedNodes()
        );

        ImGui::Text(
            "Waypoints locaux : %d",
            static_cast<int>(navigationController.GetTemporalPath().size())
        );

        ImGui::Text(
            "Raison echec : %s",
            navigationController.GetTemporalFailureReason()
        );

        const TemporalPlanningDiagnostics& temporalDiagnostics =
            navigationController.GetTemporalDiagnostics();

        ImGui::Separator();
        ImGui::Text(
            "Menaces : %d | venant de l'arriere : %d",
            temporalDiagnostics.threatsDetected,
            temporalDiagnostics.rearThreatsDetected
        );
        if (temporalDiagnostics.minimumTimeToCollision >= 0.0f)
        {
            ImGui::Text(
                "TTC minimal : %.2f s | obstacle : %u",
                temporalDiagnostics.minimumTimeToCollision,
                temporalDiagnostics.mostDangerousObstacleId
            );
        }
        else
        {
            ImGui::Text("TTC minimal : aucun");
        }
        ImGui::Text(
            "Declencheur : %s",
            temporalDiagnostics.lastTriggerReason.c_str()
        );
        ImGui::Text(
            "Recherches : %llu (%llu succes / %llu echecs)",
            static_cast<unsigned long long>(
                temporalDiagnostics.searchesRun),
            static_cast<unsigned long long>(
                temporalDiagnostics.searchesSucceeded),
            static_cast<unsigned long long>(
                temporalDiagnostics.searchesFailed)
        );
        ImGui::Text(
            "Derniere recherche : %.2f ms | %d noeuds",
            temporalDiagnostics.lastSearchDurationMilliseconds,
            temporalDiagnostics.lastExpandedNodes
        );
        ImGui::Text(
            "Voisins rejetes : %d | collisions : %d",
            temporalDiagnostics.lastRejectedNeighbors,
            temporalDiagnostics.lastRejectedCollisions
        );
        ImGui::Text(
            "Rejets horizon : %d | hors fenetre : %d",
            temporalDiagnostics.lastRejectedTimeHorizon,
            temporalDiagnostics.lastRejectedLocalWindow
        );
        std::string recentSearchTimes;
        for (const float duration :
             temporalDiagnostics.recentSearchDurationsMilliseconds)
        {
            if (!recentSearchTimes.empty())
                recentSearchTimes += ", ";
            recentSearchTimes += std::to_string(duration);
        }
        ImGui::TextWrapped(
            "Durees recentes (ms) : %s",
            recentSearchTimes.empty()
                ? "aucune"
                : recentSearchTimes.c_str()
        );
        ImGui::Text(
            "Noeuds cumules : %llu | echec : %s",
            static_cast<unsigned long long>(
                temporalDiagnostics.totalExpandedNodes),
            temporalDiagnostics.lastSearchFailureReason.c_str()
        );


        // ========================================
        // COMMANDES
        // ========================================

        ImGui::Spacing();
        ImGui::Separator();

        if (ImGui::CollapsingHeader(
            "COMMANDES",
            ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::BulletText("Fleches : deplacer");
            ImGui::BulletText(
                "En mode cible, l'auto-navigation reste active"
            );
            ImGui::BulletText("WASD / E-Q : camera");
            ImGui::BulletText("Molette : zoom");
            ImGui::BulletText("Clic droit : rotation camera");
        }
    }

    ImGui::End();
}
