
#pragma once

#include "raylib.h"
#include "../core/Config.h"

#include <vector>
#include <cstdint>
#include <cstddef>
#include <unordered_map>

// ============================================
// COORDONNEES D'UNE CELLULE
// ============================================

struct GridCell
{
    int x = 0;
    int z = 0;
};

// ============================================
// ETAT D'UNE CELLULE
// ============================================

enum class CellState
{
    Unknown,
    Free,
    Blocked
};

// ============================================
// DESCRIPTION D'UNE MODIFICATION
// ============================================

struct GridCellChange
{
    GridCell cell;

    bool stateChanged = false;
    bool costChanged = false;
};

// ============================================
// GRILLE DE NAVIGATION
// ============================================

class GridMap
{
private:
    int cellsX = 0;
    int cellsZ = 0;

    float cellSize = 1.0f;

    float originX = 0.0f;
    float originZ = 0.0f;

    std::vector<CellState> cells;
    std::vector<float> traversalCosts;

    std::vector<GridCellChange> changedCells;
    std::vector<std::uint8_t> changedFlags;

    std::size_t GetIndex(int x, int z) const;

    void MarkCellChanged(
        std::size_t index,
        const GridCell& cell,
        bool stateChanged,
        bool costChanged
    );

    // Nombre d'obstacles statiques occupant chaque cellule.
    std::vector<std::uint32_t> staticOccupancyCounts;

    // Cellules occupées par chaque obstacle statique.
    std::unordered_map<std::uint32_t, std::vector<GridCell>>
        staticObstacleCells;

public:
    explicit GridMap(const TerrainConfig& terrain);

    void Rebuild(const TerrainConfig& terrain);

    // Dimensions
    int GetWidth() const;
    int GetHeight() const;
    float GetCellSize() const;

    // Coordonnees
    bool IsInside(int x, int z) const;
    bool IsInside(const GridCell& cell) const;

    bool WorldToCell(float worldX, float worldZ, GridCell& cell) const;
    Vector2 CellToWorld(const GridCell& cell) const;

    // Etat des cellules
    CellState GetCellState(const GridCell& cell) const;
    bool SetCellState(const GridCell& cell, CellState state);

    void Clear(CellState state = CellState::Unknown);
    bool HasCellChanged(const GridCell& cell, CellState newState) const;

    // Modifications en attente
    const std::vector<GridCellChange>& GetChangedCells() const;
    void ClearChangedCells();

    // Couts de deplacement
    float GetTraversalCost(const GridCell& cell) const;
    bool SetTraversalCost(const GridCell& cell, float cost);

    bool IsTraversable(const GridCell& cell) const;

    // Géométrie des obstacles rectangulaires
    bool GetCellsIntersectingRectangle(
        float centerX,
        float centerZ,
        float width,
        float depth,
        float safetyMargin,
        std::vector<GridCell>& result
    ) const;

    // Gestion des obstacles statiques
    bool UpdateStaticObstacle(
        std::uint32_t obstacleId,
        float centerX,
        float centerZ,
        float width,
        float depth,
        float safetyMargin
    );

    bool RemoveStaticObstacle(std::uint32_t obstacleId);
};
