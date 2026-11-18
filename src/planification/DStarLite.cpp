
#include "DStarLite.h"

#include <cmath>
#include <limits>
#include <algorithm>

// ============================================
// INDEX D'UNE CELLULE
// ============================================

std::size_t DStarLite::GetIndex(const GridCell& cell) const
{
    return static_cast<std::size_t>(
        cell.z * width + cell.x
    );
}

// ============================================
// LECTURE DE G
// ============================================

float DStarLite::GetG(const GridCell& cell) const
{
    if (!grid || !grid->IsInside(cell))
        return std::numeric_limits<float>::infinity();

    return g[GetIndex(cell)];
}

// ============================================
// LECTURE DE RHS
// ============================================

float DStarLite::GetRhs(const GridCell& cell) const
{
    if (!grid || !grid->IsInside(cell))
        return std::numeric_limits<float>::infinity();

    return rhs[GetIndex(cell)];
}

// ============================================
// MODIFICATION DE G
// ============================================

void DStarLite::SetG(const GridCell& cell, float value)
{
    if (!grid || !grid->IsInside(cell))
        return;

    g[GetIndex(cell)] = value;
}

// ============================================
// MODIFICATION DE RHS
// ============================================

void DStarLite::SetRhs(const GridCell& cell, float value)
{
    if (!grid || !grid->IsInside(cell))
        return;

    rhs[GetIndex(cell)] = value;
}


// ============================================
// HEURISTIQUE OCTILE
// ============================================

float DStarLite::Heuristic(
    const GridCell& a,
    const GridCell& b
) const
{
    const int dx = std::abs(a.x - b.x);
    const int dz = std::abs(a.z - b.z);

    const int diagonal = std::min(dx, dz);
    const int straight = std::max(dx, dz) - diagonal;

    constexpr float SQRT_TWO = 1.41421356237f;

    return static_cast<float>(straight)
        + static_cast<float>(diagonal) * SQRT_TWO;
}

// ============================================
// COUT D'UN DEPLACEMENT
// ============================================

float DStarLite::GetCost(
    const GridCell& from,
    const GridCell& to
) const
{
    if (!grid || !grid->IsInside(from) || !grid->IsInside(to))
        return std::numeric_limits<float>::infinity();

    if (!grid->IsTraversable(from) || !grid->IsTraversable(to))
        return std::numeric_limits<float>::infinity();

    const int dx = std::abs(to.x - from.x);
    const int dz = std::abs(to.z - from.z);

    // Un déplacement doit être adjacent.
    if (dx > 1 || dz > 1 || (dx == 0 && dz == 0))
        return std::numeric_limits<float>::infinity();

    // Une diagonale ne doit pas couper un coin bloqué.
    if (dx == 1 && dz == 1)
    {
        GridCell sideX = { to.x, from.z };
        GridCell sideZ = { from.x, to.z };

        if (!grid->IsTraversable(sideX) ||
            !grid->IsTraversable(sideZ))
        {
            return std::numeric_limits<float>::infinity();
        }
    }

    constexpr float SQRT_TWO = 1.41421356237f;

    const float distance =
        (dx == 1 && dz == 1) ? SQRT_TWO : 1.0f;

    const float destinationCost =
        grid->GetTraversalCost(to);

    return distance * destinationCost;
}

// ============================================
// CALCUL DE LA CLE D'UNE CELLULE
// ============================================

DStarKey DStarLite::CalculateKey(const GridCell& cell) const
{
    const float gValue = GetG(cell);
    const float rhsValue = GetRhs(cell);

    const float minimum = std::min(gValue, rhsValue);

    return
    {
        minimum + Heuristic(start, cell) + km,
        minimum
    };
}

// ============================================
// COMPARAISON DES CLES
// ============================================

bool DStarLite::KeyLess(
    const DStarKey& a,
    const DStarKey& b
) const
{
    if (a.first < b.first)
        return true;

    if (a.first > b.first)
        return false;

    return a.second < b.second;
}

// ============================================
// EGALITE DES CLES
// ============================================

bool DStarLite::KeyEqual(
    const DStarKey& a,
    const DStarKey& b
) const
{
    return a.first == b.first &&
           a.second == b.second;
}


// ============================================
// VERIFICATION D'UN DEPLACEMENT DIAGONAL
// ============================================

bool DStarLite::CanMoveDiagonally(
    const GridCell& from,
    const GridCell& to
) const
{
    if (!grid ||
        !grid->IsInside(from) ||
        !grid->IsInside(to))
    {
        return false;
    }

    const int dx = std::abs(to.x - from.x);
    const int dz = std::abs(to.z - from.z);

    if (dx != 1 || dz != 1)
        return false;

    GridCell sideX = { to.x, from.z };
    GridCell sideZ = { from.x, to.z };

    return grid->IsTraversable(sideX) &&
           grid->IsTraversable(sideZ);
}

// ============================================
// SUCCESSEURS
// ============================================

std::vector<GridCell> DStarLite::GetSuccessors(
    const GridCell& cell
) const
{
    std::vector<GridCell> successors;

    if (!grid || !grid->IsInside(cell))
        return successors;

    // 8 directions : cardinales et diagonales.
    constexpr int directions[8][2] =
    {
        {  1,  0 },
        { -1,  0 },
        {  0,  1 },
        {  0, -1 },
        {  1,  1 },
        {  1, -1 },
        { -1,  1 },
        { -1, -1 }
    };

    successors.reserve(8);

    for (const auto& direction : directions)
    {
        GridCell next =
        {
            cell.x + direction[0],
            cell.z + direction[1]
        };

        if (!grid->IsTraversable(next))
            continue;

        if (direction[0] != 0 && direction[1] != 0)
        {
            if (!CanMoveDiagonally(cell, next))
                continue;
        }

        successors.push_back(next);
    }

    return successors;
}

// ============================================
// PREDECESSEURS
// ============================================

std::vector<GridCell> DStarLite::GetPredecessors(
    const GridCell& cell
) const
{
    std::vector<GridCell> predecessors;

    if (!grid || !grid->IsInside(cell))
        return predecessors;

    constexpr int directions[8][2] =
    {
        {  1,  0 },
        { -1,  0 },
        {  0,  1 },
        {  0, -1 },
        {  1,  1 },
        {  1, -1 },
        { -1,  1 },
        { -1, -1 }
    };

    predecessors.reserve(8);

    for (const auto& direction : directions)
    {
        GridCell previous =
        {
            cell.x + direction[0],
            cell.z + direction[1]
        };

        if (!grid->IsTraversable(previous))
            continue;

        // Le déplacement précédent -> cellule doit être valide.
        if (direction[0] != 0 && direction[1] != 0)
        {
            if (!CanMoveDiagonally(previous, cell))
                continue;
        }

        predecessors.push_back(previous);
    }

    return predecessors;
}


// ============================================
// MISE A JOUR D'UN SOMMET
// ============================================

void DStarLite::UpdateVertex(const GridCell& cell)
{
    if (!grid || !grid->IsInside(cell))
        return;

    const std::size_t index = GetIndex(cell);

    // La destination possède toujours rhs = 0.
    // La destination possède rhs = 0
    // uniquement si elle reste traversable.
    if (cell.x == goal.x && cell.z == goal.z)
    {
        SetRhs(
            cell,
            grid->IsTraversable(goal)
                ? 0.0f
                : std::numeric_limits<float>::infinity()
        );
    }
    else
    {
        float bestCost = std::numeric_limits<float>::infinity();

        const std::vector<GridCell> successors =
            GetSuccessors(cell);

        for (const GridCell& successor : successors)
        {
            const float candidate =
                GetCost(cell, successor) + GetG(successor);

            bestCost = std::min(bestCost, candidate);
        }

        SetRhs(cell, bestCost);
    }

    // Invalide les anciennes entrées de cette cellule.
    ++queueVersions[index];

    // Une cellule incohérente doit être dans la file.
    if (GetG(cell) != GetRhs(cell))
    {
        openList.push(
            {
                cell,
                CalculateKey(cell),
                queueVersions[index]
            }
        );
    }
}


// ============================================
// CALCUL DU PLUS COURT CHEMIN
// ============================================

void DStarLite::ComputeShortestPath()
{
    if (!grid)
        return;

    while (true)
    {
        // Suppression logique des anciennes entrées.
        while (!openList.empty())
        {
            const DStarQueueEntry& entry = openList.top();

            const std::size_t index = GetIndex(entry.cell);

            if (entry.version != queueVersions[index])
            {
                openList.pop();
                continue;
            }

            break;
        }

        const DStarKey startKey = CalculateKey(start);

        const bool startInconsistent =
            GetRhs(start) != GetG(start);

        if (openList.empty())
            break;

        const DStarQueueEntry current = openList.top();

        // Condition d'arrêt de D* Lite.
        if (!KeyLess(current.key, startKey) &&
            !startInconsistent)
        {
            break;
        }

        openList.pop();

        const GridCell u = current.cell;

        const DStarKey oldKey = current.key;
        const DStarKey newKey = CalculateKey(u);

        // La priorité a changé depuis l'insertion.
        if (!KeyEqual(oldKey, newKey))
        {
            openList.push(
                {
                    u,
                    newKey,
                    current.version
                }
            );

            continue;
        }

        // Cas 1 : le coût devient meilleur.
        if (GetG(u) > GetRhs(u))
        {
            SetG(u, GetRhs(u));

            const std::vector<GridCell> predecessors =
                GetPredecessors(u);

            for (const GridCell& predecessor : predecessors)
                UpdateVertex(predecessor);
        }
        // Cas 2 : le coût précédent n'est plus valide.
        else
        {
            SetG(
                u,
                std::numeric_limits<float>::infinity()
            );

            UpdateVertex(u);

            const std::vector<GridCell> predecessors =
                GetPredecessors(u);

            for (const GridCell& predecessor : predecessors)
                UpdateVertex(predecessor);
        }
    }
}


// ============================================
// INITIALISATION DE D* LITE
// ============================================

bool DStarLite::Initialize(
    const GridMap& gridMap,
    const GridCell& initialStart,
    const GridCell& initialGoal
)
{
    // Vérification des dimensions.
    if (gridMap.GetWidth() <= 0 ||
        gridMap.GetHeight() <= 0)
    {
        return false;
    }

    // Vérification des positions.
    if (!gridMap.IsInside(initialStart) ||
        !gridMap.IsInside(initialGoal))
    {
        return false;
    }

    if (!gridMap.IsTraversable(initialStart) ||
        !gridMap.IsTraversable(initialGoal))
    {
        return false;
    }

    grid = &gridMap;

    width = gridMap.GetWidth();
    height = gridMap.GetHeight();

    start = initialStart;
    lastStart = initialStart;
    goal = initialGoal;

    km = 0.0f;

    const std::size_t cellCount =
        static_cast<std::size_t>(width) *
        static_cast<std::size_t>(height);

    const float infinity =
        std::numeric_limits<float>::infinity();

    g.assign(cellCount, infinity);
    rhs.assign(cellCount, infinity);

    queueVersions.assign(cellCount, 0);

    openList = {};

    // La destination est le point de départ
    // de la propagation inverse.
    SetRhs(goal, 0.0f);

    UpdateVertex(goal);

    return true;
}

// ============================================
// MISE A JOUR DU DEPART
// ============================================

bool DStarLite::UpdateStart(const GridCell& newStart)
{
    if (!grid || !grid->IsInside(newStart))
        return false;

    if (!grid->IsTraversable(newStart))
        return false;

    // Rien à faire si le départ n'a pas changé.
    if (newStart.x == start.x &&
        newStart.z == start.z)
    {
        return true;
    }

    // Adaptation des priorités au déplacement.
    km += Heuristic(lastStart, newStart);

    lastStart = newStart;
    start = newStart;

    return true;
}

// ============================================
// MISE A JOUR DE LA DESTINATION
// ============================================

bool DStarLite::UpdateGoal(const GridCell& newGoal)
{
    if (!grid || !grid->IsInside(newGoal))
        return false;

    if (!grid->IsTraversable(newGoal))
        return false;

    // Une nouvelle destination nécessite
    // une nouvelle propagation.
    goal = newGoal;

    km = 0.0f;
    lastStart = start;

    const float infinity =
        std::numeric_limits<float>::infinity();

    std::fill(g.begin(), g.end(), infinity);
    std::fill(rhs.begin(), rhs.end(), infinity);

    std::fill(
        queueVersions.begin(),
        queueVersions.end(),
        0
    );

    openList = {};

    SetRhs(goal, 0.0f);

    UpdateVertex(goal);

    return true;
}


// ============================================
// NOTIFICATION D'UNE MODIFICATION DE CELLULE
// ============================================

void DStarLite::NotifyCellChanged(const GridCell& cell)
{
    if (!grid || !grid->IsInside(cell))
        return;

    // Une modification peut affecter :
    // - les déplacements vers cette cellule ;
    // - les déplacements depuis cette cellule ;
    // - les diagonales passant par cette cellule.
    //
    // On actualise donc son voisinage 3x3.

    for (int dz = -1; dz <= 1; ++dz)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            GridCell affected =
            {
                cell.x + dx,
                cell.z + dz
            };

            if (!grid->IsInside(affected))
                continue;

            UpdateVertex(affected);
        }
    }
}


// ============================================
// CALCUL OU REPARATION DU CHEMIN
// ============================================

bool DStarLite::ComputePath()
{
    if (!grid)
        return false;

    if (!grid->IsInside(start) ||
        !grid->IsInside(goal))
    {
        return false;
    }

    if (!grid->IsTraversable(start) ||
        !grid->IsTraversable(goal))
    {
        return false;
    }

    ComputeShortestPath();

    return std::isfinite(GetG(start));
}


// ============================================
// EXTRACTION DU CHEMIN
// ============================================

std::vector<GridCell> DStarLite::GetPath() const
{
    std::vector<GridCell> path;

    if (!grid)
        return path;

    if (!grid->IsInside(start) ||
        !grid->IsInside(goal))
    {
        return path;
    }

    if (!grid->IsTraversable(start) ||
        !grid->IsTraversable(goal))
    {
        return path;
    }

    if (!std::isfinite(GetG(start)))
        return path;

    // Le départ est déjà la destination.
    path.push_back(start);

    if (start.x == goal.x && start.z == goal.z)
        return path;

    const std::size_t cellCount =
        static_cast<std::size_t>(width) *
        static_cast<std::size_t>(height);

    std::vector<bool> visited(cellCount, false);

    GridCell current = start;

    visited[GetIndex(current)] = true;

    // Protection contre les boucles.
    for (std::size_t step = 0; step < cellCount; ++step)
    {
        const std::vector<GridCell> successors =
            GetSuccessors(current);

        float bestCost =
            std::numeric_limits<float>::infinity();

        GridCell bestSuccessor = current;

        bool foundSuccessor = false;

        for (const GridCell& successor : successors)
        {
            const std::size_t index = GetIndex(successor);

            if (visited[index])
                continue;

            const float candidate =
                GetCost(current, successor) +
                GetG(successor);

            if (!std::isfinite(candidate))
                continue;

            if (candidate < bestCost)
            {
                bestCost = candidate;
                bestSuccessor = successor;
                foundSuccessor = true;
            }
        }

        if (!foundSuccessor)
        {
            path.clear();
            return path;
        }

        current = bestSuccessor;

        path.push_back(current);

        visited[GetIndex(current)] = true;

        if (current.x == goal.x &&
            current.z == goal.z)
        {
            return path;
        }
    }

    // Destination non atteinte dans la limite autorisée.
    path.clear();

    return path;
}

// ============================================
// ETAT D'INITIALISATION
// ============================================

bool DStarLite::IsInitialized() const
{
    return grid != nullptr &&
           width > 0 &&
           height > 0;
}

// ============================================
// VALIDITE DU CHEMIN
// ============================================

bool DStarLite::HasValidPath() const
{
    if (!IsInitialized())
        return false;

    if (!grid->IsTraversable(start) ||
        !grid->IsTraversable(goal))
    {
        return false;
    }

    return std::isfinite(GetG(start));
}
