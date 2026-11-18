#pragma once

#include "GridMap.h"
#include "../monde/Monde.h"

#include <cstdint>
#include <unordered_set>

// ============================================
// RESULTAT DE SYNCHRONISATION
// ============================================

enum class GridSyncResult
{
    NoChange,
    IncrementalUpdate,
    FullRebuild
};

// ============================================
// SYNCHRONISATEUR WORLD -> GRIDMAP
// ============================================

class GridSynchronizer
{
private:
    std::uint64_t lastNavigationRevision =
        static_cast<std::uint64_t>(-1);

    TerrainConfig lastTerrain;

    bool hasPreviousTerrain = false;

    std::unordered_set<std::uint32_t> synchronizedObstacleIds;

    bool HasTerrainGeometryChanged(
        const TerrainConfig& terrain
    ) const;

    float lastSafetyMargin = 0.0f;
    bool hasPreviousSafetyMargin = false;

public:
    GridSyncResult Synchronize(
        const World& world,
        GridMap& grid,
        float safetyMargin
    );

    void Reset();
};