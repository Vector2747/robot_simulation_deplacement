
#include "WorldRender.h"

#include <algorithm>

void DrawWorld3D(
    const World& world,
    const Robot& robot,
    const Target& target,
    const Camera3D& camera,
    const NavigationController& navigationController
)
{
    // ============================================
    // CAMERA
    // ============================================

    BeginMode3D(camera);

    // ============================================
    // TERRAIN
    // ============================================

    const TerrainConfig& terrain = world.GetTerrain();

    const float terrainWidth =
        terrain.cellsX * terrain.cellSize;

    const float terrainDepth =
        terrain.cellsZ * terrain.cellSize;

    DrawPlane(
        Vector3{
            0.0f,
            terrain.groundY,
            0.0f
        },
        Vector2{
            terrainWidth,
            terrainDepth
        },
        Color{ 55, 65, 55, 255 }
    );

    if (terrain.showGrid)
    {
        const int gridSlices = 40;

        const float gridSpacing =
            terrainWidth / gridSlices;

        DrawGrid(
            gridSlices,
            gridSpacing
        );
    }

    // ============================================
    // OBSTACLES
    // ============================================

    for (const Obstacle& obstacle : world.GetObstacles())
    {
        const Color obstacleColor =
            obstacle.isMoving ? ORANGE : GRAY;

        DrawCube(
            obstacle.position,
            obstacle.size.x,
            obstacle.size.y,
            obstacle.size.z,
            obstacleColor
        );

        DrawCubeWires(
            obstacle.position,
            obstacle.size.x,
            obstacle.size.y,
            obstacle.size.z,
            DARKGRAY
        );
    }

    // ============================================
    // ROBOT
    // ============================================

    DrawSphere(
        robot.GetPosition(),
        robot.GetRadius(),
        BLUE
    );

    // ============================================
    // CHEMIN GLOBAL D* LITE
    // ============================================

    const GridMap* globalGrid =
        navigationController.GetGlobalGrid();

    const std::vector<GridCell>& globalPath =
        navigationController.GetGlobalPath();

    if (globalGrid != nullptr && globalPath.size() >= 2)
    {
        const float pathHeight = terrain.groundY + 0.08f;

        for (std::size_t i = 1; i < globalPath.size(); ++i)
        {
            const Vector2 previous =
                globalGrid->CellToWorld(globalPath[i - 1]);

            const Vector2 current =
                globalGrid->CellToWorld(globalPath[i]);

            DrawLine3D(
                Vector3{
                    previous.x,
                    pathHeight,
                    previous.y
                },
                Vector3{
                    current.x,
                    pathHeight,
                    current.y
                },
                GREEN
            );
        }
    }

    // ============================================
    // CHEMIN TEMPOREL - TEMPORAL A*
    // ============================================

    const LocalGrid* localGrid = navigationController.GetLocalGrid();
    const std::vector<TemporalWaypoint>& temporalPath =
        navigationController.GetTemporalPath();

    if (localGrid != nullptr &&
        localGrid->IsInitialized() &&
        temporalPath.size() >= 2)
    {
        const float temporalPathHeight = terrain.groundY + 0.13f;

        for (std::size_t i = 1; i < temporalPath.size(); ++i)
        {
            const Vector2 previous =
                localGrid->CellToWorld(temporalPath[i - 1].cell);

            const Vector2 current =
                localGrid->CellToWorld(temporalPath[i].cell);

            DrawLine3D(
                { previous.x, temporalPathHeight, previous.y },
                { current.x, temporalPathHeight, current.y },
                Color{ 0, 220, 255, 255 }
            );

            // Marqueur sur chaque waypoint.
            DrawSphere(
                { current.x, temporalPathHeight, current.y },
                0.08f,
                Color{ 0, 220, 255, 255 }
            );
        }
    }

    // ============================================
    // GRILLE LOCALE - JAUNE
    // ============================================

    /*const LocalGrid* localGrid =
        navigationController.GetLocalGrid();*/

    if (localGrid != nullptr && localGrid->IsInitialized())
    {
        const int width = localGrid->GetWidth();
        const int height = localGrid->GetHeight();

        const float cellSize = localGrid->GetCellSize();

        const float gridY = terrain.groundY + 0.06f;

        // Centre de la première cellule.
        const Vector2 firstCell =
            localGrid->CellToWorld({ 0, 0 });

        // Coins de la fenêtre locale.
        const float startX = firstCell.x - cellSize * 0.5f;
        const float startZ = firstCell.y - cellSize * 0.5f;

        const float endX = startX + width * cellSize;
        const float endZ = startZ + height * cellSize;

        // ============================================
        // ETAT DES CELLULES LOCALES
        // ============================================

        for (int z = 0; z < height; ++z)
        {
            for (int x = 0; x < width; ++x)
            {
                const GridCell cell = { x, z };

                const CellState state =
                    localGrid->GetCellState(cell);

                // Ne pas afficher les cellules inconnues.
                if (state == CellState::Unknown)
                    continue;

                if (state == CellState::Blocked)
                {
                    const Vector2 center =
                        localGrid->CellToWorld(cell);

                    DrawCube(
                        {
                            center.x,
                            gridY,
                            center.y
                        },
                        cellSize * 0.90f,
                        0.035f,
                        cellSize * 0.90f,
                        Color{ 220, 55, 55, 150 }
                    );
                }
            }
        }

        // Lignes parallèles à X.
        for (int z = 0; z <= height; ++z)
        {
            const float worldZ = startZ + z * cellSize;

            DrawLine3D(
                { startX, gridY, worldZ },
                { endX, gridY, worldZ },
                YELLOW
            );
        }

        // Lignes parallèles à Z.
        for (int x = 0; x <= width; ++x)
        {
            const float worldX = startX + x * cellSize;

            DrawLine3D(
                { worldX, gridY, startZ },
                { worldX, gridY, endZ },
                YELLOW
            );
        }
    }

    // ============================================
    // PREDICTIONS DES OBSTACLES MOBILES - ORANGE
    // ============================================

    if (localGrid != nullptr && localGrid->IsInitialized())
    {
        const auto& predictedPoints =
            localGrid->GetPredictedObstaclePoints();

        for (const PredictedObstaclePoint& point : predictedPoints)
        {
            // La position actuelle est déjà représentée
            // par l'obstacle dans le monde.
            if (point.time <= 0.001f)
                continue;

            const float progress =
                point.time / 0.5f;

            const float clampedProgress =
                std::clamp(progress, 0.0f, 1.0f);

            const unsigned char alpha =
                static_cast<unsigned char>(
                    220.0f - 120.0f * clampedProgress
                );

            const Vector3 predictionPosition = {
                point.position.x,
                terrain.groundY + 0.12f,
                point.position.z
            };

            // Emplacement futur de l'obstacle.
            DrawCube(
                predictionPosition,
                point.size.x * 0.85f,
                0.04f,
                point.size.z * 0.85f,
                Color{ 255, 140, 0, alpha }
            );

            // Contour de la position prédite.
            DrawCubeWires(
                predictionPosition,
                point.size.x,
                0.08f,
                point.size.z,
                Color{ 255, 165, 0, alpha }
            );

            // Centre de la prédiction.
            DrawSphere(
                predictionPosition,
                0.12f,
                Color{ 255, 100, 0, alpha }
            );
        }
    }

    // ============================================
    // TRAJECTOIRES PREDITES DES OBSTACLES
    // ============================================

    if (localGrid != nullptr && localGrid->IsInitialized())
    {
        const auto& predictedPoints =
            localGrid->GetPredictedObstaclePoints();

        for (std::size_t i = 1; i < predictedPoints.size(); ++i)
        {
            const PredictedObstaclePoint& previous =
                predictedPoints[i - 1];

            const PredictedObstaclePoint& current =
                predictedPoints[i];

            // Relier uniquement deux prédictions
            // appartenant au même obstacle.
            if (previous.obstacleId != current.obstacleId)
                continue;

            if (current.time <= previous.time)
                continue;

            DrawLine3D(
                {
                    previous.position.x,
                    terrain.groundY + 0.18f,
                    previous.position.z
                },
                {
                    current.position.x,
                    terrain.groundY + 0.18f,
                    current.position.z
                },
                ORANGE
            );
        }
    }

    // ============================================
    // TARGET
    // ============================================

    DrawCylinder(
        target.GetPosition(),
        0.6f,
        0.6f,
        1.2f,
        16,
        RED
    );

    // ============================================
    // FIN DU RENDU 3D
    // ============================================

    EndMode3D();
}
