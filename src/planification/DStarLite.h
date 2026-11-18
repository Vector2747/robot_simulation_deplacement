
#pragma once

#include "GridMap.h"

#include <vector>
#include <queue>
#include <cstddef>
#include <cstdint>

// ============================================
// CLE DE PRIORITE D* LITE
// ============================================

struct DStarKey
{
    float first = 0.0f;
    float second = 0.0f;
};

// ============================================
// ELEMENT DE LA FILE DE PRIORITE
// ============================================


struct DStarQueueEntry
{
    GridCell cell;
    DStarKey key;
    std::uint64_t version = 0;
};


// ============================================
// COMPARATEUR DE PRIORITE
// ============================================

struct DStarKeyComparator
{
    bool operator()(
        const DStarQueueEntry& a,
        const DStarQueueEntry& b
    ) const
    {
        if (a.key.first != b.key.first)
            return a.key.first > b.key.first;

        return a.key.second > b.key.second;
    }
};

// ============================================
// PLANIFICATEUR D* LITE
// ============================================

class DStarLite
{
private:

    const GridMap* grid = nullptr;

    int width = 0;
    int height = 0;

    GridCell start;
    GridCell lastStart;
    GridCell goal;

    float km = 0.0f;

    // Valeurs fondamentales de D* Lite
    std::vector<float> g;
    std::vector<float> rhs;

    // File de priorité
    std::priority_queue<
        DStarQueueEntry,
        std::vector<DStarQueueEntry>,
        DStarKeyComparator
    > openList;

    // ========================================
    // OUTILS INTERNES
    // ========================================

    std::size_t GetIndex(const GridCell& cell) const;

    float GetG(const GridCell& cell) const;
    float GetRhs(const GridCell& cell) const;

    void SetG(const GridCell& cell, float value);
    void SetRhs(const GridCell& cell, float value);

    float Heuristic(
        const GridCell& a,
        const GridCell& b
    ) const;

    float GetCost(
        const GridCell& from,
        const GridCell& to
    ) const;

    DStarKey CalculateKey(const GridCell& cell) const;

    bool KeyLess(
        const DStarKey& a,
        const DStarKey& b
    ) const;

    bool KeyEqual(
        const DStarKey& a,
        const DStarKey& b
    ) const;

    // ========================================
    // VOISINAGE A 8 DIRECTIONS
    // ========================================

    std::vector<GridCell> GetSuccessors(
        const GridCell& cell
    ) const;

    std::vector<GridCell> GetPredecessors(
        const GridCell& cell
    ) const;

    bool CanMoveDiagonally(
        const GridCell& from,
        const GridCell& to
    ) const;

    // ========================================
    // MECANISME D* LITE
    // ========================================

    void UpdateVertex(const GridCell& cell);

    void ComputeShortestPath();
    
    // Versions des entrées de la file de priorité
    std::vector<std::uint64_t> queueVersions;

public:

    DStarLite() = default;

    bool Initialize(
        const GridMap& gridMap,
        const GridCell& initialStart,
        const GridCell& initialGoal
    );

    bool UpdateStart(const GridCell& newStart);

    bool UpdateGoal(const GridCell& newGoal);

    void NotifyCellChanged(const GridCell& cell);

    bool ComputePath();

    std::vector<GridCell> GetPath() const;

    bool IsInitialized() const;

    bool HasValidPath() const;
};
