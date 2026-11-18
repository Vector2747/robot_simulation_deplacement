#pragma once

#include "../planification/GridMap.h"

#include <vector>
#include <cstdint>

class World;

// Un échantillon de trajectoire prédite.
struct PredictedObstaclePoint
{
    std::uint32_t obstacleId = 0;

    // Position prédite de l'obstacle.
    Vector3 position = { 0.0f, 0.0f, 0.0f };

    // Dimensions de l'obstacle.
    Vector3 size = { 1.0f, 1.0f, 1.0f };

    // Instant auquel cette position est prévue.
    float time = 0.0f;
};

// Trajectoire prédite complète d'un obstacle.
// Les échantillons sont conservés dans l'ordre temporel.
struct CachedObstacleTrajectory
{
    std::uint32_t obstacleId = 0;

    std::vector<PredictedObstaclePoint> samples;
};

class LocalGrid
{
private:

    // Dimensions de la fenêtre locale
    int cellsX = 40;
    int cellsZ = 40;

    // Taille réelle d'une cellule
    float cellSize = 1.0f;

    // Origine de la fenêtre dans le monde
    float originX = 0.0f;
    float originZ = 0.0f;

    // Position du centre de la fenêtre
    float centerX = 0.0f;
    float centerZ = 0.0f;

    // État des cellules locales
    std::vector<CellState> cells;

    // Indique si la fenêtre a été initialisée
    bool initialized = false;

    std::size_t GetIndex(int x, int z) const;

    std::vector<std::uint8_t> cellsNeedingSync;

    // Couche indépendante des obstacles mobiles.
    // 0 = libre, 1 = occupé.
    std::vector<std::uint8_t> dynamicBlocked;

    // Accumulateur de temps pour les mises à jour dynamiques.
    float dynamicUpdateAccumulator = 0.0f;

    // Intervalle de rafraîchissement : 25 Hz.
    float dynamicUpdateInterval = 0.04f;
    std::uint64_t lastDynamicWorldRevision =
        static_cast<std::uint64_t>(-1);

    // Coût de traversée de chaque cellule locale.
    std::vector<float> traversalCosts;

    // Points des trajectoires prédites des obstacles mobiles.
    std::vector<PredictedObstaclePoint> predictedObstaclePoints;

    // Identifiants des obstacles mobiles pertinents
    // pour la fenêtre locale (test balayé temporaire).
    std::vector<std::uint32_t> relevantObstacleIds;

    // Liste temporaire réutilisée à chaque actualisation.
    std::vector<std::uint32_t> detectedObstacleIds;

    // Indique si le registre a changé depuis sa dernière actualisation.
    bool relevantObstacleSetChanged = false;


    // ============================================
    // CACHE DES TRAJECTOIRES MOBILES
    // ============================================

    // Une trajectoire pré-calculée par obstacle pertinent.
    std::vector<CachedObstacleTrajectory> cachedObstacleTrajectories;

    // Temps écoulé depuis la génération du cache.
    float predictionCacheAge = 0.0f;

    // Paramètres temporels du cache.
    float predictionCacheStep = 0.1f;
    float predictionCacheHorizon = 3.2f;

    // Mode de déplacement de la fenêtre locale.
    // false = suivi classique
    // true  = déplacement par blocs avec marge
    bool stableWindowMovement = true;

public:

    LocalGrid() = default;

    // Initialise la fenêtre locale
    void Initialize(
        int width,
        int height,
        float cellSize
    );

    // Déplace la fenêtre autour du robot
    bool UpdateWindow(
        float robotX,
        float robotZ
    );

    // Reconstruit le contenu local depuis le Global Grid
    void UpdateFromGlobalGrid(
        const GridMap& globalGrid
    );

    // Conversion coordonnées monde / cellule locale
    bool WorldToCell(
        float worldX,
        float worldZ,
        GridCell& result
    ) const;

    Vector2 CellToWorld(
        const GridCell& cell
    ) const;

    // Accès aux cellules
    CellState GetCellState(
        const GridCell& cell
    ) const;

    bool SetCellState(
        const GridCell& cell,
        CellState state
    );

    // Informations générales
    int GetWidth() const;
    int GetHeight() const;
    float GetCellSize() const;
    bool IsInitialized() const;
    // Marque toutes les cellules pour une nouvelle synchronisation.
    void InvalidateAllCells();

    // Efface uniquement les informations dynamiques.
    void ClearDynamicObstacles();

    // Marque les cellules occupées par un obstacle mobile.
    void AddDynamicObstacle(
        float centerX,
        float centerZ,
        float width,
        float depth,
        float safetyMargin
    );

    // Synchronisation prédictive des obstacles mobiles.
    bool UpdateDynamicObstacles(
        const World& world,
        float dt,
        float predictionHorizon,
        float safetyMargin
    );

    // Actualise uniquement les cellules locales concernées
    // par les modifications du Global Grid.
    void UpdateFromGlobalChanges(
        const GridMap& globalGrid,
        const std::vector<GridCellChange>& changes
    );

    // Vérifie si une cellule peut être traversée.
    bool IsTraversable(const GridCell& cell) const;

    // Récupère les cellules voisines traversables.
    std::vector<GridCell> GetNeighbors(
        const GridCell& cell
    ) const;

    float GetTraversalCost(const GridCell& cell) const;

    bool SetTraversalCost(
        const GridCell& cell,
        float cost
    );

    // Récupère les points de toutes les trajectoires prédites.
    const std::vector<PredictedObstaclePoint>&
    GetPredictedObstaclePoints() const;

    CellState GetStaticCellState(const GridCell&) const;

    bool IsStaticFootprintFree(
        float worldX,
        float worldZ,
        float radius
    ) const;

    // Indique si au moins un obstacle mobile est pertinent.
    bool HasRelevantDynamicObstacles() const;

    // Indique si le registre a changé et consomme le signal.
    bool ConsumeRelevantObstacleSetChanged();

    // Accès en lecture seule aux identifiants pertinents.
    const std::vector<std::uint32_t>&
    GetRelevantObstacleIds() const;

    // Accès en lecture seule au cache des trajectoires.
    const std::vector<CachedObstacleTrajectory>&
    GetCachedObstacleTrajectories() const;

    // Âge du cache depuis sa dernière génération.
    float GetPredictionCacheAge() const;

    void SetStableWindowMovement(bool enabled)
    {
        stableWindowMovement = enabled;
    }

    bool IsStableWindowMovement() const
    {
        return stableWindowMovement;
    }
};