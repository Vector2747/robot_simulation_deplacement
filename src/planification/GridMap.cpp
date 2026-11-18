
#include "GridMap.h"

#include <cmath>
#include <limits>
#include <algorithm>
#include <utility>

// ============================================
// CONSTRUCTEUR
// ============================================

GridMap::GridMap(const TerrainConfig& terrain)
{
    Rebuild(terrain);
}

// ============================================
// RECONSTRUCTION DE LA GRILLE
// ============================================

void GridMap::Rebuild(const TerrainConfig& terrain)
{
    // Reprendre les dimensions du terrain.
    cellsX = terrain.cellsX;
    cellsZ = terrain.cellsZ;

    // Garantir des dimensions valides.
    if (cellsX < 1)
        cellsX = 1;

    if (cellsZ < 1)
        cellsZ = 1;

    // Vérifier la résolution des cellules.
    cellSize = terrain.cellSize;

    if (!std::isfinite(cellSize) || cellSize <= 0.0f)
        cellSize = 1.0f;

    // Le terrain est centré sur l'origine.
    const float terrainWidth = cellsX * cellSize;
    const float terrainDepth = cellsZ * cellSize;

    originX = -terrainWidth * 0.5f;
    originZ = -terrainDepth * 0.5f;

    cells.assign(
        static_cast<std::size_t>(cellsX) *
        static_cast<std::size_t>(cellsZ),
        CellState::Free
    );

    traversalCosts.assign(
        cells.size(),
        1.0f
    );

    changedFlags.assign(cells.size(), 0);
    changedCells.clear();

    staticOccupancyCounts.assign(cells.size(), 0);
    staticObstacleCells.clear();
}

// ============================================
// DIMENSIONS
// ============================================

int GridMap::GetWidth() const
{
    return cellsX;
}

int GridMap::GetHeight() const
{
    return cellsZ;
}

float GridMap::GetCellSize() const
{
    return cellSize;
}

// ============================================
// VERIFICATION DES COORDONNEES
// ============================================

bool GridMap::IsInside(int x, int z) const
{
    return
        x >= 0 &&
        z >= 0 &&
        x < cellsX &&
        z < cellsZ;
}

bool GridMap::IsInside(const GridCell& cell) const
{
    return IsInside(cell.x, cell.z);
}

// ============================================
// MONDE -> GRILLE
// ============================================

bool GridMap::WorldToCell(
    float worldX,
    float worldZ,
    GridCell& cell
) const
{
    if (!std::isfinite(worldX) || !std::isfinite(worldZ))
        return false;

    if (cellSize <= 0.0f)
        return false;

    const double gridX = std::floor(
        (static_cast<double>(worldX) - originX) / cellSize
    );

    const double gridZ = std::floor(
        (static_cast<double>(worldZ) - originZ) / cellSize
    );

    if (
        gridX < 0.0 ||
        gridZ < 0.0 ||
        gridX >= cellsX ||
        gridZ >= cellsZ
    )
    {
        return false;
    }

    GridCell result;

    result.x = static_cast<int>(gridX);
    result.z = static_cast<int>(gridZ);

    cell = result;

    return true;
}

// ============================================
// GRILLE -> MONDE
// ============================================

Vector2 GridMap::CellToWorld(const GridCell& cell) const
{
    if (!IsInside(cell))
    {
        const float invalid = std::numeric_limits<float>::quiet_NaN();
        return { invalid, invalid };
    }

    // Retourner le centre de la cellule.
    const float worldX =
        originX + (cell.x + 0.5f) * cellSize;

    const float worldZ =
        originZ + (cell.z + 0.5f) * cellSize;

    return { worldX, worldZ };
}


// ============================================
// INDEXATION DES CELLULES
// ============================================

std::size_t GridMap::GetIndex(int x, int z) const
{
    return
        static_cast<std::size_t>(z) *
        static_cast<std::size_t>(cellsX) +
        static_cast<std::size_t>(x);
}

// ============================================
// LECTURE DE L'ETAT D'UNE CELLULE
// ============================================

CellState GridMap::GetCellState(const GridCell& cell) const
{
    if (!IsInside(cell))
        return CellState::Unknown;

    return cells[GetIndex(cell.x, cell.z)];
}

// ============================================
// MODIFICATION DE L'ETAT D'UNE CELLULE
// ============================================


bool GridMap::SetCellState(
    const GridCell& cell,
    CellState state
)
{
    if (!IsInside(cell))
        return false;

    const std::size_t index = GetIndex(cell.x, cell.z);

    if (cells[index] == state)
        return false;

    const float oldCost = GetTraversalCost(cell);

    cells[index] = state;

    const float newCost = GetTraversalCost(cell);

    MarkCellChanged(
        index,
        cell,
        true,
        oldCost != newCost
    );

    return true;
}


// ============================================
// REINITIALISATION DE LA GRILLE
// ============================================


void GridMap::Clear(CellState state)
{
    // Supprimer toutes les associations d'obstacles.
    staticObstacleCells.clear();

    // Réinitialiser les compteurs d'occupation.
    std::fill(
        staticOccupancyCounts.begin(),
        staticOccupancyCounts.end(),
        0
    );

    // Réinitialiser les états des cellules.
    for (int z = 0; z < cellsZ; ++z)
    {
        for (int x = 0; x < cellsX; ++x)
        {
            SetCellState({ x, z }, state);
        }
    }
}


// ============================================
// DETECTION DES CHANGEMENTS
// ============================================

bool GridMap::HasCellChanged(
    const GridCell& cell,
    CellState newState
) const
{
    if (!IsInside(cell))
        return false;

    return GetCellState(cell) != newState;
}

// ============================================
// ENREGISTREMENT D'UNE MODIFICATION
// ============================================

void GridMap::MarkCellChanged(
    std::size_t index,
    const GridCell& cell,
    bool stateChanged,
    bool costChanged
)
{
    if (changedFlags[index] != 0)
    {
        for (GridCellChange& change : changedCells)
        {
            if (change.cell.x == cell.x &&
                change.cell.z == cell.z)
            {
                change.stateChanged |= stateChanged;
                change.costChanged |= costChanged;
                return;
            }
        }
    }

    changedFlags[index] = 1;

    changedCells.push_back({
        cell,
        stateChanged,
        costChanged
    });
}

// ============================================
// CONSULTATION DES MODIFICATIONS
// ============================================

const std::vector<GridCellChange>& GridMap::GetChangedCells() const
{
    return changedCells;
}

void GridMap::ClearChangedCells()
{
    for (const GridCellChange& change : changedCells)
    {
        const std::size_t index = GetIndex(
            change.cell.x,
            change.cell.z
        );

        changedFlags[index] = 0;
    }


    changedCells.clear();
}

// ============================================
// LECTURE DU COUT DE DEPLACEMENT
// ============================================


float GridMap::GetTraversalCost(const GridCell& cell) const
{
    if (!IsInside(cell))
        return std::numeric_limits<float>::infinity();

    const std::size_t index = GetIndex(cell.x, cell.z);

    if (cells[index] != CellState::Free)
        return std::numeric_limits<float>::infinity();

    return traversalCosts[index];
}


// ============================================
// MODIFICATION DU COUT DE DEPLACEMENT
// ============================================



bool GridMap::SetTraversalCost(
    const GridCell& cell,
    float cost
)
{
    if (!IsInside(cell))
        return false;

    // Le coût minimal doit rester égal à 1
    // pour préserver l'admissibilité de l'heuristique octile.
    if (!std::isfinite(cost) || cost < 1.0f)
        return false;

    const std::size_t index = GetIndex(cell.x, cell.z);

    if (traversalCosts[index] == cost)
        return false;

    traversalCosts[index] = cost;

    // Le changement de coût n'affecte le planificateur
    // que si la cellule est actuellement libre.
    if (cells[index] == CellState::Free)
    {
        MarkCellChanged(index, cell, false, true);
    }

    return true;
}



bool GridMap::IsTraversable(const GridCell& cell) const
{
    if (!IsInside(cell))
        return false;

    const std::size_t index = GetIndex(cell.x, cell.z);

    return
        cells[index] == CellState::Free &&
        std::isfinite(traversalCosts[index]);
}


// ============================================
// CELLULES INTERSECTEES PAR UN RECTANGLE
// ============================================

bool GridMap::GetCellsIntersectingRectangle(
    float centerX,
    float centerZ,
    float width,
    float depth,
    float safetyMargin,
    std::vector<GridCell>& result
) const
{
    result.clear();

    // Vérification des paramètres.
    if (
        !std::isfinite(centerX) ||
        !std::isfinite(centerZ) ||
        !std::isfinite(width) ||
        !std::isfinite(depth) ||
        !std::isfinite(safetyMargin)
    )
    {
        return false;
    }

    if (
        width <= 0.0f ||
        depth <= 0.0f ||
        safetyMargin < 0.0f ||
        cellSize <= 0.0f
    )
    {
        return false;
    }

    // Demi-dimensions avec marge de sécurité.
    const double halfWidth =
        static_cast<double>(width) * 0.5 + safetyMargin;

    const double halfDepth =
        static_cast<double>(depth) * 0.5 + safetyMargin;

    // Rectangle agrandi.
    const double minWorldX = centerX - halfWidth;
    const double maxWorldX = centerX + halfWidth;

    const double minWorldZ = centerZ - halfDepth;
    const double maxWorldZ = centerZ + halfDepth;

    // Limites physiques du terrain.
    const double terrainMinX = originX;
    const double terrainMinZ = originZ;

    const double terrainMaxX =
        terrainMinX + static_cast<double>(cellsX) * cellSize;

    const double terrainMaxZ =
        terrainMinZ + static_cast<double>(cellsZ) * cellSize;

    // Aucun contact avec le terrain.
    if (
        maxWorldX < terrainMinX ||
        minWorldX > terrainMaxX ||
        maxWorldZ < terrainMinZ ||
        minWorldZ > terrainMaxZ
    )
    {
        return true;
    }

    // Intersection du rectangle avec le terrain.
    const double clippedMinX =
        std::max(minWorldX, terrainMinX);

    const double clippedMaxX =
        std::min(maxWorldX, terrainMaxX);

    const double clippedMinZ =
        std::max(minWorldZ, terrainMinZ);

    const double clippedMaxZ =
        std::min(maxWorldZ, terrainMaxZ);

    // Conversion en coordonnées de grille.
    const int minCellX = std::clamp(
        static_cast<int>(std::floor(
            (clippedMinX - terrainMinX) / cellSize
        )),
        0,
        cellsX - 1
    );

    const int maxCellX = std::clamp(
        static_cast<int>(std::floor(
            (clippedMaxX - terrainMinX) / cellSize
        )),
        0,
        cellsX - 1
    );

    const int minCellZ = std::clamp(
        static_cast<int>(std::floor(
            (clippedMinZ - terrainMinZ) / cellSize
        )),
        0,
        cellsZ - 1
    );

    const int maxCellZ = std::clamp(
        static_cast<int>(std::floor(
            (clippedMaxZ - terrainMinZ) / cellSize
        )),
        0,
        cellsZ - 1
    );

    // Collecte des cellules concernées.
    for (int z = minCellZ; z <= maxCellZ; ++z)
    {
        for (int x = minCellX; x <= maxCellX; ++x)
        {
            result.push_back({ x, z });
        }
    }

    return true;
}

// ============================================
// MISE A JOUR D'UN OBSTACLE STATIQUE
// ============================================

bool GridMap::UpdateStaticObstacle(
    std::uint32_t obstacleId,
    float centerX,
    float centerZ,
    float width,
    float depth,
    float safetyMargin
)
{
    if (obstacleId == 0)
        return false;

    std::vector<GridCell> newCells;

    if (!GetCellsIntersectingRectangle(
        centerX,
        centerZ,
        width,
        depth,
        safetyMargin,
        newCells
    ))
    {
        return false;
    }

    // Récupérer l'ancienne empreinte.
    auto it = staticObstacleCells.find(obstacleId);

    // Si l'empreinte est identique, aucun travail nécessaire.
    if (it != staticObstacleCells.end())
    {
        const std::vector<GridCell>& oldCells = it->second;

        if (oldCells.size() == newCells.size())
        {
            bool identical = true;

            for (std::size_t i = 0; i < oldCells.size(); ++i)
            {
                if (
                    oldCells[i].x != newCells[i].x ||
                    oldCells[i].z != newCells[i].z
                )
                {
                    identical = false;
                    break;
                }
            }

            if (identical)
                return false;
        }
    }

    // Retirer l'ancienne empreinte.
    if (it != staticObstacleCells.end())
    {
        for (const GridCell& cell : it->second)
        {
            const std::size_t index =
                GetIndex(cell.x, cell.z);

            if (staticOccupancyCounts[index] > 0)
                --staticOccupancyCounts[index];

            if (staticOccupancyCounts[index] == 0)
                SetCellState(cell, CellState::Free);
        }

        staticObstacleCells.erase(it);
    }

    // Ajouter la nouvelle empreinte.
    for (const GridCell& cell : newCells)
    {
        const std::size_t index =
            GetIndex(cell.x, cell.z);

        ++staticOccupancyCounts[index];

        if (staticOccupancyCounts[index] == 1)
            SetCellState(cell, CellState::Blocked);
    }

    staticObstacleCells.emplace(
        obstacleId,
        std::move(newCells)
    );

    return true;
}


// ============================================
// SUPPRESSION D'UN OBSTACLE STATIQUE
// ============================================

bool GridMap::RemoveStaticObstacle(
    std::uint32_t obstacleId
)
{
    auto it = staticObstacleCells.find(obstacleId);

    if (it == staticObstacleCells.end())
        return false;

    // Libérer uniquement les cellules qui ne sont
    // plus occupées par aucun autre obstacle.
    for (const GridCell& cell : it->second)
    {
        const std::size_t index =
            GetIndex(cell.x, cell.z);

        if (staticOccupancyCounts[index] > 0)
            --staticOccupancyCounts[index];

        if (staticOccupancyCounts[index] == 0)
            SetCellState(cell, CellState::Free);
    }

    staticObstacleCells.erase(it);

    return true;
}