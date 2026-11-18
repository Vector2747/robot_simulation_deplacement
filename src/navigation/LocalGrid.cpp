
#include "LocalGrid.h"
#include "../monde/Monde.h"

#include <cmath>
#include <algorithm>
#include <utility>

// ============================================
// INITIALISATION
// ============================================

void LocalGrid::Initialize(
    int width,
    int height,
    float requestedCellSize
)
{
    if (width <= 0 ||
        height <= 0 ||
        !std::isfinite(requestedCellSize) ||
        requestedCellSize <= 0.0f)
    {
        initialized = false;
        cells.clear();
        return;
    }

    cellsX = width;
    cellsZ = height;
    cellSize = requestedCellSize;
    lastDynamicWorldRevision =
        static_cast<std::uint64_t>(-1);

    cells.assign(
        static_cast<std::size_t>(cellsX) * cellsZ,
        CellState::Unknown
    );

    cellsNeedingSync.assign(cells.size(), 1);

    dynamicBlocked.assign(cells.size(), 0);

    traversalCosts.assign(cells.size(), 1.0f);

    relevantObstacleIds.clear();
    detectedObstacleIds.clear();
    relevantObstacleSetChanged = false;
    predictedObstaclePoints.clear();

    cachedObstacleTrajectories.clear();
    predictionCacheAge = 0.0f;

    centerX = 0.0f;
    centerZ = 0.0f;

    originX = centerX - (cellsX * cellSize) * 0.5f;
    originZ = centerZ - (cellsZ * cellSize) * 0.5f;

    initialized = true;
}

// ============================================
// INDEXATION
// ============================================

std::size_t LocalGrid::GetIndex(int x, int z) const
{
    return static_cast<std::size_t>(z) * cellsX + x;
}

// ============================================
// DEPLACEMENT DE LA FENETRE
// ============================================


bool LocalGrid::UpdateWindow(
    float robotX,
    float robotZ
)
{
    if (!initialized ||
        !std::isfinite(robotX) ||
        !std::isfinite(robotZ))
    {
        return false;
    }

    float newOriginX = originX;
    float newOriginZ = originZ;

    int shiftX = 0;
    int shiftZ = 0;

    // ========================================
    // MODE CLASSIQUE : SUIVI CELLULE PAR CELLULE
    // ========================================

    if (!stableWindowMovement)
    {
        const float snappedX =
            std::floor(robotX / cellSize) * cellSize;

        const float snappedZ =
            std::floor(robotZ / cellSize) * cellSize;

        newOriginX =
            snappedX - (cellsX / 2) * cellSize;

        newOriginZ =
            snappedZ - (cellsZ / 2) * cellSize;

        if (newOriginX == originX &&
            newOriginZ == originZ)
        {
            return false;
        }

        shiftX = static_cast<int>(
            std::round((newOriginX - originX) / cellSize)
        );

        shiftZ = static_cast<int>(
            std::round((newOriginZ - originZ) / cellSize)
        );
    }

    // ========================================
    // MODE STABLE : DEPLACEMENT PAR BLOCS
    // ========================================

    else
    {
        const int marginX = std::max(1, cellsX / 4);
        const int marginZ = std::max(1, cellsZ / 4);

        const int chunkX = std::min(8, marginX);
        const int chunkZ = std::min(8, marginZ);

        const int robotCellX = static_cast<int>(
            std::floor((robotX - originX) / cellSize)
        );

        const int robotCellZ = static_cast<int>(
            std::floor((robotZ - originZ) / cellSize)
        );

        // Déplacement horizontal.

        if (robotCellX < marginX)
        {
            const int needed = marginX - robotCellX;

            const int chunks =
                (needed + chunkX - 1) / chunkX;

            shiftX = -chunks * chunkX;
        }
        else if (robotCellX >= cellsX - marginX)
        {
            const int needed =
                robotCellX - (cellsX - marginX) + 1;

            const int chunks =
                (needed + chunkX - 1) / chunkX;

            shiftX = chunks * chunkX;
        }

        // Déplacement vertical (axe Z).

        if (robotCellZ < marginZ)
        {
            const int needed = marginZ - robotCellZ;

            const int chunks =
                (needed + chunkZ - 1) / chunkZ;

            shiftZ = -chunks * chunkZ;
        }
        else if (robotCellZ >= cellsZ - marginZ)
        {
            const int needed =
                robotCellZ - (cellsZ - marginZ) + 1;

            const int chunks =
                (needed + chunkZ - 1) / chunkZ;

            shiftZ = chunks * chunkZ;
        }

        if (shiftX == 0 && shiftZ == 0)
            return false;

        newOriginX = originX + shiftX * cellSize;
        newOriginZ = originZ + shiftZ * cellSize;
    }

    // ========================================
    // CREATION DES NOUVEAUX TABLEAUX
    // ========================================

    const std::size_t totalCells =
        static_cast<std::size_t>(cellsX) * cellsZ;

    std::vector<CellState> shiftedCells(
        totalCells,
        CellState::Unknown
    );

    std::vector<std::uint8_t> shiftedSync(
        totalCells,
        1
    );

    std::vector<float> shiftedCosts(
        totalCells,
        1.0f
    );

    // ========================================
    // CONSERVATION DES CELLULES COMMUNES
    // ========================================

    for (int z = 0; z < cellsZ; ++z)
    {
        for (int x = 0; x < cellsX; ++x)
        {
            const int newX = x - shiftX;
            const int newZ = z - shiftZ;

            if (newX < 0 || newX >= cellsX ||
                newZ < 0 || newZ >= cellsZ)
            {
                continue;
            }

            const std::size_t oldIndex = GetIndex(x, z);

            const std::size_t newIndex =
                static_cast<std::size_t>(newZ) * cellsX + newX;

            shiftedCells[newIndex] = cells[oldIndex];

            shiftedSync[newIndex] =
                cellsNeedingSync[oldIndex];

            shiftedCosts[newIndex] =
                traversalCosts[oldIndex];
        }
    }

    // ========================================
    // APPLICATION DU DEPLACEMENT
    // ========================================

    cells = std::move(shiftedCells);

    cellsNeedingSync = std::move(shiftedSync);

    traversalCosts = std::move(shiftedCosts);

    // ========================================
    // INVALIDATION DES DONNEES DYNAMIQUES
    // ========================================

    cachedObstacleTrajectories.clear();

    predictedObstaclePoints.clear();

    detectedObstacleIds.clear();

    predictionCacheAge = 0.0f;

    relevantObstacleIds.clear();

    relevantObstacleSetChanged = true;

    // Forcer une actualisation dynamique.
    dynamicUpdateAccumulator = dynamicUpdateInterval;

    // ========================================
    // MISE A JOUR DE LA FENETRE
    // ========================================

    originX = newOriginX;
    originZ = newOriginZ;

    centerX =
        originX + (cellsX * cellSize) * 0.5f;

    centerZ =
        originZ + (cellsZ * cellSize) * 0.5f;

    return true;
}


// ============================================
// SYNCHRONISATION AVEC LE GLOBAL GRID
// ============================================


void LocalGrid::UpdateFromGlobalGrid(
    const GridMap& globalGrid
)
{
    if (!initialized)
        return;

    for (int z = 0; z < cellsZ; ++z)
    {
        for (int x = 0; x < cellsX; ++x)
        {
            const std::size_t index = GetIndex(x, z);

            if (cellsNeedingSync[index] == 0)
                continue;

            const GridCell localCell = { x, z };
            const Vector2 worldPosition = CellToWorld(localCell);

            GridCell globalCell;

            if (globalGrid.WorldToCell(
                worldPosition.x,
                worldPosition.y,
                globalCell))
            {
                const CellState globalState =
                    globalGrid.GetCellState(globalCell);

                cells[index] = globalState;

                if (globalState == CellState::Free)
                {
                    const float cost =
                        globalGrid.GetTraversalCost(globalCell);

                    if (std::isfinite(cost) && cost >= 1.0f)
                        traversalCosts[index] = cost;
                }
                else
                {
                    traversalCosts[index] = 1.0f;
                }
            }
            else
            {
                cells[index] = CellState::Unknown;
                traversalCosts[index] = 1.0f;
            }

            cellsNeedingSync[index] = 0;
        }
    }
}


// ============================================
// CONVERSION MONDE -> CELLULE
// ============================================

bool LocalGrid::WorldToCell(
    float worldX,
    float worldZ,
    GridCell& result
) const
{
    if (!initialized ||
        !std::isfinite(worldX) ||
        !std::isfinite(worldZ))
    {
        return false;
    }

    const int x = static_cast<int>(
        std::floor((worldX - originX) / cellSize)
    );

    const int z = static_cast<int>(
        std::floor((worldZ - originZ) / cellSize)
    );

    if (x < 0 || x >= cellsX ||
        z < 0 || z >= cellsZ)
    {
        return false;
    }

    result = { x, z };

    return true;
}

// ============================================
// CONVERSION CELLULE -> MONDE
// ============================================

Vector2 LocalGrid::CellToWorld(
    const GridCell& cell
) const
{
    if (!initialized ||
        cell.x < 0 || cell.x >= cellsX ||
        cell.z < 0 || cell.z >= cellsZ)
    {
        return { 0.0f, 0.0f };
    }

    return {
        originX + (cell.x + 0.5f) * cellSize,
        originZ + (cell.z + 0.5f) * cellSize
    };
}

// ============================================
// ACCES AUX CELLULES
// ============================================

CellState LocalGrid::GetCellState(
    const GridCell& cell
) const
{
    if (!initialized ||
        cell.x < 0 || cell.x >= cellsX ||
        cell.z < 0 || cell.z >= cellsZ)
    {
        return CellState::Unknown;
    }

    const std::size_t index = GetIndex(cell.x, cell.z);

    // Un obstacle mobile rend la cellule bloquée,
    // sans modifier la couche statique.
    if (dynamicBlocked[index] != 0)
        return CellState::Blocked;

    return cells[index];
}

bool LocalGrid::SetCellState(
    const GridCell& cell,
    CellState state
)
{
    if (!initialized ||
        cell.x < 0 || cell.x >= cellsX ||
        cell.z < 0 || cell.z >= cellsZ)
    {
        return false;
    }

    cells[GetIndex(cell.x, cell.z)] = state;

    return true;
}

// ============================================
// INFORMATIONS GENERALES
// ============================================

int LocalGrid::GetWidth() const
{
    return cellsX;
}

int LocalGrid::GetHeight() const
{
    return cellsZ;
}

float LocalGrid::GetCellSize() const
{
    return cellSize;
}

bool LocalGrid::IsInitialized() const
{
    return initialized;
}

void LocalGrid::InvalidateAllCells()
{
    if (!initialized)
        return;

    std::fill(
        cellsNeedingSync.begin(),
        cellsNeedingSync.end(),
        1
    );
}

// ============================================
// GESTION DES OBSTACLES DYNAMIQUES
// ============================================

void LocalGrid::ClearDynamicObstacles()
{
    if (!initialized)
        return;

    std::fill(
        dynamicBlocked.begin(),
        dynamicBlocked.end(),
        0
    );
}

void LocalGrid::AddDynamicObstacle(
    float centerX,
    float centerZ,
    float width,
    float depth,
    float safetyMargin
)
{
    if (!initialized ||
        !std::isfinite(centerX) ||
        !std::isfinite(centerZ) ||
        !std::isfinite(width) ||
        !std::isfinite(depth) ||
        !std::isfinite(safetyMargin) ||
        width <= 0.0f ||
        depth <= 0.0f ||
        safetyMargin < 0.0f)
    {
        return;
    }

    const float halfWidth = width * 0.5f + safetyMargin;
    const float halfDepth = depth * 0.5f + safetyMargin;

    const float minX = centerX - halfWidth;
    const float maxX = centerX + halfWidth;
    const float minZ = centerZ - halfDepth;
    const float maxZ = centerZ + halfDepth;

    const int firstX = std::max(
        0,
        static_cast<int>(std::floor((minX - originX) / cellSize))
    );

    const int lastX = std::min(
        cellsX - 1,
        static_cast<int>(std::floor((maxX - originX) / cellSize))
    );

    const int firstZ = std::max(
        0,
        static_cast<int>(std::floor((minZ - originZ) / cellSize))
    );

    const int lastZ = std::min(
        cellsZ - 1,
        static_cast<int>(std::floor((maxZ - originZ) / cellSize))
    );

    for (int z = firstZ; z <= lastZ; ++z)
    {
        for (int x = firstX; x <= lastX; ++x)
        {
            const float cellMinX = originX + x * cellSize;
            const float cellMaxX = cellMinX + cellSize;

            const float cellMinZ = originZ + z * cellSize;
            const float cellMaxZ = cellMinZ + cellSize;

            const bool intersects =
                cellMaxX >= minX &&
                cellMinX <= maxX &&
                cellMaxZ >= minZ &&
                cellMinZ <= maxZ;

            if (intersects)
            {
                dynamicBlocked[GetIndex(x, z)] = 1;
            }
        }
    }
}

// ============================================
// SYNCHRONISATION DES OBSTACLES MOBILES
// ============================================



bool LocalGrid::UpdateDynamicObstacles(
    const World& world,
    float dt,
    float predictionHorizon,
    float safetyMargin
)
{
    if (!initialized ||
        !std::isfinite(dt) ||
        !std::isfinite(predictionHorizon) ||
        !std::isfinite(safetyMargin) ||
        dt <= 0.0f ||
        predictionHorizon < 0.0f ||
        safetyMargin < 0.0f ||
        !std::isfinite(dynamicUpdateInterval) ||
        dynamicUpdateInterval <= 0.0f)
    {
        return false;
    }

    // Le cache vieillit entre deux actualisations.
    predictionCacheAge += dt;

    if (!std::isfinite(predictionCacheAge))
        predictionCacheAge = predictionCacheHorizon;

    dynamicUpdateAccumulator += dt;
    const std::uint64_t worldRevision =
        world.GetNavigationRevision();
    const bool worldObstacleSetChanged =
        worldRevision != lastDynamicWorldRevision;

    if (dynamicUpdateAccumulator < dynamicUpdateInterval &&
        !worldObstacleSetChanged)
    {
        return false;
    }

    if (worldObstacleSetChanged)
    {
        dynamicUpdateAccumulator = 0.0f;
        lastDynamicWorldRevision = worldRevision;
    }
    else
    {
        dynamicUpdateAccumulator = std::fmod(
            dynamicUpdateAccumulator,
            dynamicUpdateInterval
        );
    }

    ClearDynamicObstacles();
    predictedObstaclePoints.clear();

    // Préparer la détection du nouveau registre.
    // clear() conserve la capacité mémoire déjà allouée.
    detectedObstacleIds.clear();

    constexpr int predictionSamples = 10;

    // Limites réelles de la fenêtre locale.
    const float localMinX = originX;
    const float localMaxX = originX + cellsX * cellSize;
    const float localMinZ = originZ;
    const float localMaxZ = originZ + cellsZ * cellSize;

    // Vérifie l'intersection de deux rectangles AABB.
    const auto IntersectsLocalWindow =
        [&](float centerX,
            float centerZ,
            float halfWidth,
            float halfDepth) -> bool
    {
        return
            centerX + halfWidth >= localMinX &&
            centerX - halfWidth <= localMaxX &&
            centerZ + halfDepth >= localMinZ &&
            centerZ - halfDepth <= localMaxZ;
    };

    for (const Obstacle& obstacle : world.GetObstacles())
    {
        if (!obstacle.isMoving || obstacle.id == 0)
            continue;

        // Vérification des données avant tout traitement.

        // Vérification des données.
        if (!std::isfinite(obstacle.position.x) ||
            !std::isfinite(obstacle.position.z) ||
            !std::isfinite(obstacle.size.x) ||
            !std::isfinite(obstacle.size.z) ||
            !std::isfinite(obstacle.velocity.x) ||
            !std::isfinite(obstacle.velocity.y) ||
            obstacle.size.x <= 0.0f ||
            obstacle.size.z <= 0.0f)
        {
            continue;
        }

        const float halfWidth =
            obstacle.size.x * 0.5f + safetyMargin;

        const float halfDepth =
            obstacle.size.z * 0.5f + safetyMargin;

        // ------------------------------------------------
        // 1. FILTRAGE RAPIDE : ZONE DE DEPLACEMENT POSSIBLE
        // ------------------------------------------------

        // Distance maximale que l'obstacle peut parcourir
        // sur chaque axe pendant l'horizon.
        //
        // Cette estimation reste conservative même si
        // l'obstacle rebondit sur les limites du terrain.

        const float reachX =
            std::abs(obstacle.velocity.x) * predictionHorizon;

        const float reachZ =
            std::abs(obstacle.velocity.y) * predictionHorizon;

        const float sweptHalfWidth = halfWidth + reachX;
        const float sweptHalfDepth = halfDepth + reachZ;

        const bool potentiallyRelevant =
            IntersectsLocalWindow(
                obstacle.position.x,
                obstacle.position.z,
                sweptHalfWidth,
                sweptHalfDepth
            );

        // Aucun contact possible avec la fenêtre locale.
        if (!potentiallyRelevant)
            continue;

        // L'obstacle entre dans le registre uniquement
        // s'il peut atteindre la fenêtre locale.
        detectedObstacleIds.push_back(obstacle.id);

        // ------------------------------------------------
        // 2. POSITION ACTUELLE
        // ------------------------------------------------

        predictedObstaclePoints.push_back({
            obstacle.id,
            obstacle.position,
            obstacle.size,
            0.0f
        });

        // AddDynamicObstacle limite automatiquement
        // les cellules marquées à la fenêtre locale.
        AddDynamicObstacle(
            obstacle.position.x,
            obstacle.position.z,
            obstacle.size.x,
            obstacle.size.z,
            safetyMargin
        );

        // ------------------------------------------------
        // 3. POSITIONS FUTURES
        // ------------------------------------------------

        for (int sample = 1; sample <= predictionSamples; ++sample)
        {
            const float futureTime =
                predictionHorizon *
                static_cast<float>(sample) /
                predictionSamples;

            const Obstacle predicted =
                world.GetPredictedObstacleState(
                    obstacle,
                    futureTime
                );

            // Ne conserver que les positions qui
            // intersectent réellement la fenêtre locale.
            const bool intersects =
                IntersectsLocalWindow(
                    predicted.position.x,
                    predicted.position.z,
                    halfWidth,
                    halfDepth
                );

            if (!intersects)
                continue;

            predictedObstaclePoints.push_back({
                obstacle.id,
                predicted.position,
                predicted.size,
                futureTime
            });
        }
    }

    // Trier et supprimer les éventuels doublons.
    // Le registre est ainsi toujours ordonné par identifiant.
    std::sort(
        detectedObstacleIds.begin(),
        detectedObstacleIds.end()
    );

    detectedObstacleIds.erase(
        std::unique(
            detectedObstacleIds.begin(),
            detectedObstacleIds.end()
        ),
        detectedObstacleIds.end()
    );

    // Détecter une entrée ou une sortie d'obstacle.
    relevantObstacleSetChanged =
        detectedObstacleIds != relevantObstacleIds;

    // Remplacer l'ancien registre.
    relevantObstacleIds.swap(detectedObstacleIds);

    // ============================================
    // CONSTRUCTION DU CACHE DES TRAJECTOIRES
    // ============================================

    const float cacheStep = predictionCacheStep;
    const float cacheHorizon = predictionCacheHorizon;

    if (!std::isfinite(cacheStep) ||
        !std::isfinite(cacheHorizon) ||
        cacheStep <= 0.0f ||
        cacheHorizon <= 0.0f)
    {
        cachedObstacleTrajectories.clear();
        predictionCacheAge = cacheHorizon;
        return true;
    }

    // Nombre d'intervalles temporels.
    // Avec 3,2 s et un pas de 0,1 s : 32 intervalles,
    // donc 33 échantillons en comptant t = 0.
    const int sampleIntervals = static_cast<int>(
        std::ceil(cacheHorizon / cacheStep)
    );

    const auto& obstacles = world.GetObstacles();

    // Construire ou actualiser une trajectoire par ID.
    for (const std::uint32_t obstacleId : relevantObstacleIds)
    {
        const auto obstacleIt = std::find_if(
            obstacles.begin(),
            obstacles.end(),
            [obstacleId](const Obstacle& obstacle)
            {
                return obstacle.id == obstacleId &&
                    obstacle.isMoving;
            }
        );

        if (obstacleIt == obstacles.end())
            continue;

        const Obstacle& obstacle = *obstacleIt;

        // Le cache est trié par identifiant.
        auto cacheIt = std::lower_bound(
            cachedObstacleTrajectories.begin(),
            cachedObstacleTrajectories.end(),
            obstacleId,
            [](const CachedObstacleTrajectory& entry,
            std::uint32_t id)
            {
                return entry.obstacleId < id;
            }
        );

        // Créer l'entrée si elle n'existe pas.
        if (cacheIt == cachedObstacleTrajectories.end() ||
            cacheIt->obstacleId != obstacleId)
        {
            CachedObstacleTrajectory newEntry;
            newEntry.obstacleId = obstacleId;

            cacheIt = cachedObstacleTrajectories.insert(
                cacheIt,
                std::move(newEntry)
            );
        }

        // Réutiliser la capacité mémoire de la trajectoire.
        auto& samples = cacheIt->samples;
        samples.clear();

        samples.reserve(
            static_cast<std::size_t>(sampleIntervals + 1)
        );

        // Échantillon initial : état actuel de l'obstacle.
        samples.push_back({
            obstacle.id,
            obstacle.position,
            obstacle.size,
            0.0f
        });

        // Tous les échantillons futurs sont conservés,
        // même lorsque l'obstacle sort temporairement
        // de la fenêtre locale.
        for (int sample = 1; sample <= sampleIntervals; ++sample)
            {
                const float futureTime = std::min(
                    static_cast<float>(sample) * cacheStep,
                    cacheHorizon
                );

                const Obstacle predicted =
                    world.GetPredictedObstacleState(
                        obstacle,
                        futureTime
                    );

                samples.push_back({
                    obstacle.id,
                    predicted.position,
                    predicted.size,
                    futureTime
                });
            }
    }

    // Supprimer les trajectoires des obstacles qui ne sont
    // plus présents dans le registre local.
    cachedObstacleTrajectories.erase(
        std::remove_if(
            cachedObstacleTrajectories.begin(),
            cachedObstacleTrajectories.end(),
            [this](const CachedObstacleTrajectory& entry)
            {
                return !std::binary_search(
                    this->relevantObstacleIds.begin(),
                    this->relevantObstacleIds.end(),
                    entry.obstacleId
                );
            }
        ),
        cachedObstacleTrajectories.end()
    );

    // Le cache vient d'être reconstruit depuis l'état
    // actuel du monde.
    predictionCacheAge = 0.0f;

    return true;
}



// ============================================
// SYNCHRONISATION DES CELLULES MODIFIEES
// ============================================


void LocalGrid::UpdateFromGlobalChanges(
    const GridMap& globalGrid,
    const std::vector<GridCellChange>& changes
)
{
    if (!initialized || changes.empty())
        return;

    for (const GridCellChange& change : changes)
    {
        if (!globalGrid.IsInside(change.cell))
            continue;

        const Vector2 worldPosition =
            globalGrid.CellToWorld(change.cell);

        GridCell localCell;

        if (!WorldToCell(
            worldPosition.x,
            worldPosition.y,
            localCell))
        {
            continue;
        }

        const std::size_t index =
            GetIndex(localCell.x, localCell.z);

        const CellState globalState =
            globalGrid.GetCellState(change.cell);

        cells[index] = globalState;

        if (globalState == CellState::Free)
        {
            const float cost =
                globalGrid.GetTraversalCost(change.cell);

            if (std::isfinite(cost) && cost >= 1.0f)
                traversalCosts[index] = cost;
        }
        else
        {
            traversalCosts[index] = 1.0f;
        }

        cellsNeedingSync[index] = 0;
    }
}



// ============================================
// VERIFICATION DE TRAVERSABILITE
// ============================================

bool LocalGrid::IsTraversable(const GridCell& cell) const
{
    // Les cellules hors de la fenêtre sont interdites.
    if (!initialized ||
        cell.x < 0 || cell.x >= cellsX ||
        cell.z < 0 || cell.z >= cellsZ)
    {
        return false;
    }

    // Seules les cellules libres sont traversables.
    return GetCellState(cell) == CellState::Free;
}

// ============================================
// RECHERCHE DES VOISINS
// ============================================

std::vector<GridCell> LocalGrid::GetNeighbors(
    const GridCell& cell
) const
{
    std::vector<GridCell> neighbors;

    if (!IsTraversable(cell))
        return neighbors;

    // 8 directions : 4 cardinales et 4 diagonales.
    constexpr int directions[8][2] =
    {
        {  0, -1 },
        {  0,  1 },
        { -1,  0 },
        {  1,  0 },
        { -1, -1 },
        {  1, -1 },
        { -1,  1 },
        {  1,  1 }
    };

    for (const auto& direction : directions)
    {
        const int dx = direction[0];
        const int dz = direction[1];

        GridCell neighbor =
        {
            cell.x + dx,
            cell.z + dz
        };

        if (!IsTraversable(neighbor))
            continue;

        // Interdire les diagonales qui coupent
        // le coin de deux cellules bloquées.
        if (dx != 0 && dz != 0)
        {
            GridCell sideX = {
                cell.x + dx,
                cell.z
            };

            GridCell sideZ = {
                cell.x,
                cell.z + dz
            };

            if (!IsTraversable(sideX) ||
                !IsTraversable(sideZ))
            {
                continue;
            }
        }

        neighbors.push_back(neighbor);
    }

    return neighbors;
}

// ============================================
// COUTS DE TRAVERSEE
// ============================================

float LocalGrid::GetTraversalCost(
    const GridCell& cell
) const
{
    if (!initialized ||
        cell.x < 0 || cell.x >= cellsX ||
        cell.z < 0 || cell.z >= cellsZ)
    {
        return 0.0f;
    }

    if (!IsTraversable(cell))
        return 0.0f;

    return traversalCosts[GetIndex(cell.x, cell.z)];
}

bool LocalGrid::SetTraversalCost(
    const GridCell& cell,
    float cost
)
{
    if (!initialized ||
        cell.x < 0 || cell.x >= cellsX ||
        cell.z < 0 || cell.z >= cellsZ ||
        !std::isfinite(cost) ||
        cost < 1.0f)
    {
        return false;
    }

    traversalCosts[GetIndex(cell.x, cell.z)] = cost;

    return true;
}

// ============================================
// ACCES AUX TRAJECTOIRES PREDITES
// ============================================

const std::vector<PredictedObstaclePoint>&
LocalGrid::GetPredictedObstaclePoints() const
{
    return predictedObstaclePoints;
}

CellState LocalGrid::GetStaticCellState(
    const GridCell& cell
) const
{
    if (!initialized ||
        cell.x < 0 ||
        cell.z < 0 ||
        cell.x >= cellsX ||
        cell.z >= cellsZ)
    {
        return CellState::Unknown;
    }

    return cells[GetIndex(cell.x, cell.z)];
}


bool LocalGrid::IsStaticFootprintFree(
    float worldX,
    float worldZ,
    float radius
) const
{
    if (!initialized ||
        !std::isfinite(worldX) ||
        !std::isfinite(worldZ) ||
        !std::isfinite(radius) ||
        radius < 0.0f)
    {
        return false;
    }

    // Déterminer les cellules couvrant le disque.
    GridCell minCell;
    GridCell maxCell;

    if (!WorldToCell(
        worldX - radius,
        worldZ - radius,
        minCell
    ))
    {
        return false;
    }

    if (!WorldToCell(
        worldX + radius,
        worldZ + radius,
        maxCell
    ))
    {
        return false;
    }

    const float halfCell = cellSize * 0.5f;
    const float radiusSquared = radius * radius;

    // Vérifier uniquement les cellules intersectant le disque.
    for (int z = minCell.z; z <= maxCell.z; ++z)
    {
        for (int x = minCell.x; x <= maxCell.x; ++x)
        {
            GridCell cell = { x, z };

            const Vector2 cellCenter = CellToWorld(cell);

            const float closestX = std::clamp(
                worldX,
                cellCenter.x - halfCell,
                cellCenter.x + halfCell
            );

            const float closestZ = std::clamp(
                worldZ,
                cellCenter.y - halfCell,
                cellCenter.y + halfCell
            );

            const float dx = worldX - closestX;
            const float dz = worldZ - closestZ;

            if (dx * dx + dz * dz <= radiusSquared)
            {
                if (GetStaticCellState(cell) != CellState::Free)
                {
                    return false;
                }
            }
        }
    }

    return true;
}

bool LocalGrid::HasRelevantDynamicObstacles() const
{
    return !relevantObstacleIds.empty();
}


bool LocalGrid::ConsumeRelevantObstacleSetChanged()
{
    const bool changed = relevantObstacleSetChanged;

    relevantObstacleSetChanged = false;

    return changed;
}


const std::vector<std::uint32_t>&
LocalGrid::GetRelevantObstacleIds() const
{
    return relevantObstacleIds;
}

const std::vector<CachedObstacleTrajectory>&
LocalGrid::GetCachedObstacleTrajectories() const
{
    return cachedObstacleTrajectories;
}

float LocalGrid::GetPredictionCacheAge() const
{
    return predictionCacheAge;
}