#include "GridSynchronizer.h"

#include <cmath>
#include <utility>

// ============================================
// VERIFICATION DE LA GEOMETRIE DU TERRAIN
// ============================================

bool GridSynchronizer::HasTerrainGeometryChanged(
    const TerrainConfig& terrain
) const
{
    if (!hasPreviousTerrain)
        return true;

    return
        terrain.cellsX != lastTerrain.cellsX ||
        terrain.cellsZ != lastTerrain.cellsZ ||
        terrain.cellSize != lastTerrain.cellSize;
}


// ============================================
// SYNCHRONISATION WORLD -> GRIDMAP
// ============================================

GridSyncResult GridSynchronizer::Synchronize(
    const World& world,
    GridMap& grid,
    float safetyMargin
)
{
    // Vérifier la marge de sécurité.
    if (!std::isfinite(safetyMargin) || safetyMargin < 0.0f)
        return GridSyncResult::NoChange;

    const TerrainConfig& terrain = world.GetTerrain();

    const std::uint64_t currentRevision =
        world.GetNavigationRevision();

    const bool revisionChanged =
        currentRevision != lastNavigationRevision;

    const bool safetyMarginChanged =
        !hasPreviousSafetyMargin ||
        safetyMargin != lastSafetyMargin;

    // Rien n'a changé : aucune synchronisation nécessaire.
    if (
        !revisionChanged &&
        !safetyMarginChanged &&
        hasPreviousTerrain
    )
    {
        return GridSyncResult::NoChange;
    }

    // Déterminer si la géométrie de la grille doit être reconstruite.
    const bool geometryChanged =
        HasTerrainGeometryChanged(terrain);

    GridSyncResult result = GridSyncResult::NoChange;

    // ========================================
    // RECONSTRUCTION COMPLETE
    // ========================================

    if (geometryChanged)
    {
        grid.Rebuild(terrain);

        synchronizedObstacleIds.clear();

        result = GridSyncResult::FullRebuild;
    }

    // ========================================
    // RECUPERATION DES OBSTACLES STATIQUES
    // ========================================

    std::unordered_set<std::uint32_t> currentObstacleIds;

    for (const Obstacle& obstacle : world.GetObstacles())
    {
        // Les obstacles mobiles sont gérés localement.
        if (obstacle.isMoving)
            continue;

        currentObstacleIds.insert(obstacle.id);

        const bool updated = grid.UpdateStaticObstacle(
            obstacle.id,
            obstacle.position.x,
            obstacle.position.z,
            obstacle.size.x,
            obstacle.size.z,
            safetyMargin
        );

        if (updated && result == GridSyncResult::NoChange)
        {
            result = GridSyncResult::IncrementalUpdate;
        }
    }

    // ========================================
    // SUPPRESSION DES OBSTACLES DISPARUS
    // ========================================

    for (const std::uint32_t obstacleId : synchronizedObstacleIds)
    {
        if (currentObstacleIds.find(obstacleId) ==
            currentObstacleIds.end())
        {
            const bool removed =
                grid.RemoveStaticObstacle(obstacleId);

            if (removed && result == GridSyncResult::NoChange)
            {
                result = GridSyncResult::IncrementalUpdate;
            }
        }
    }

    // ========================================
    // MEMORISATION DE L'ETAT SYNCHRONISE
    // ========================================

    synchronizedObstacleIds = std::move(currentObstacleIds);

    lastNavigationRevision = currentRevision;

    lastTerrain = terrain;
    hasPreviousTerrain = true;

    lastSafetyMargin = safetyMargin;
    hasPreviousSafetyMargin = true;

    return result;
}


// ============================================
// REINITIALISATION DU SYNCHRONISEUR
// ============================================

void GridSynchronizer::Reset()
{
    lastNavigationRevision =
        static_cast<std::uint64_t>(-1);

    hasPreviousTerrain = false;
    hasPreviousSafetyMargin = false;

    lastSafetyMargin = 0.0f;

    synchronizedObstacleIds.clear();
}