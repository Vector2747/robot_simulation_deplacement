
#include "LocalPlanner.h"
#include "../monde/Monde.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

// ============================================
// CONFIGURATION
// ============================================

void LocalPlanner::Configure(
    float maximumSpeed,
    float radius
)
{
    if (std::isfinite(maximumSpeed) && maximumSpeed >= 0.0f)
        maxSpeed = maximumSpeed;

    if (std::isfinite(radius) && radius > 0.0f)
        robotRadius = radius;
}

// ============================================
// OUTILS GEOMETRIQUES
// ============================================

namespace
{
    constexpr float PI_VALUE = 3.14159265358979323846f;
    constexpr float EPSILON = 0.000001f;

    float Length(const Vector2& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    Vector2 Normalize(const Vector2& v)
    {
        const float length = Length(v);

        if (length < EPSILON)
            return { 0.0f, 0.0f };

        return {
            v.x / length,
            v.y / length
        };
    }

    bool SegmentIntersectsAABB(
        const Vector2& start,
        const Vector2& end,
        float halfWidth,
        float halfDepth
    )
    {
        float tMin = 0.0f;
        float tMax = 1.0f;

        auto ClipAxis = [&](
            float origin,
            float delta,
            float minimum,
            float maximum
        )
        {
            if (std::abs(delta) < EPSILON)
                return origin >= minimum && origin <= maximum;

            float t1 = (minimum - origin) / delta;
            float t2 = (maximum - origin) / delta;

            if (t1 > t2)
                std::swap(t1, t2);

            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);

            return tMin <= tMax;
        };

        return ClipAxis(
            start.x,
            end.x - start.x,
            -halfWidth,
            halfWidth
        )
        &&
        ClipAxis(
            start.y,
            end.y - start.y,
            -halfDepth,
            halfDepth
        );
    }

    float DistanceToAABB(
        const Vector2& point,
        float halfWidth,
        float halfDepth
    )
    {
        const float dx = std::max(
            0.0f,
            std::abs(point.x) - halfWidth
        );

        const float dz = std::max(
            0.0f,
            std::abs(point.y) - halfDepth
        );

        return std::sqrt(dx * dx + dz * dz);
    }

}

// ============================================
// CALCUL DU CONTROLE LOCAL
// ============================================

Vector2 LocalPlanner::CalculateVelocity(
    const Vector3& robotPosition,
    const Vector2& desiredVelocity,
    const Vector3& targetPosition,
    const std::vector<GridCell>& globalPath,
    const GridMap& globalGrid,
    float dt,
    const LocalGrid& localGrid,
    const World& world,
    bool dynamicThreatActive,
    bool holdPositionWithoutGlobalPath
)
{
    candidates.clear();
    lastDiagnostics = {};
    (void)world;

    // ----------------------------------------
    // 1. VALIDATION
    // ----------------------------------------

    if (!localGrid.IsInitialized() ||
        !std::isfinite(robotPosition.x) ||
        !std::isfinite(robotPosition.z) ||
        !std::isfinite(targetPosition.x) ||
        !std::isfinite(targetPosition.z) ||
        !std::isfinite(desiredVelocity.x) ||
        !std::isfinite(desiredVelocity.y))
    {
        previousVelocity = { 0.0f, 0.0f };
        return previousVelocity;
    }

    if (!std::isfinite(dt) || dt <= 0.0f)
        return previousVelocity;

    if (dynamicThreatActive)
    {
        preferredSideHoldTime = 0.0f;
    }
    else
    {
        preferredSideHoldTime += dt;
        if (preferredSideHoldTime > 0.35f)
            preferredAvoidanceSide = 0;
    }

    const float desiredSpeed = Length(desiredVelocity);

    const float limitedSpeed =
        holdPositionWithoutGlobalPath && dynamicThreatActive
        ? maxSpeed
        : std::min(desiredSpeed, maxSpeed);

    Vector2 desiredDirection = Normalize(desiredVelocity);
    if (holdPositionWithoutGlobalPath &&
        dynamicThreatActive &&
        Length(desiredDirection) <= EPSILON)
    {
        desiredDirection = Normalize(previousVelocity);
        if (Length(desiredDirection) <= EPSILON)
            desiredDirection = { 1.0f, 0.0f };
    }

    const Vector2 robotStart = {
        robotPosition.x,
        robotPosition.z
    };

    const Vector2 target = {
        targetPosition.x,
        targetPosition.z
    };

    const float initialTargetDistance = Length({
        target.x - robotStart.x,
        target.y - robotStart.y
    });

    // ----------------------------------------
    // 2. PARAMETRES DE RECHERCHE
    // ----------------------------------------

    const float horizon = std::max(
        0.5f,
        predictionHorizon
    );

    const float timeStep = std::max(
        0.025f,
        predictionTimeStep
    );

    const int predictionSteps = std::max(
        1,
        static_cast<int>(std::ceil(horizon / timeStep))
    );

    constexpr int beamWidth = 8;
    constexpr int trajectorySegments = 6;

    const int stepsPerSegment = std::max(
        1,
        (predictionSteps + trajectorySegments - 1)
            / trajectorySegments
    );

    
    // ----------------------------------------
    // 3. PRECALCUL DES OBSTACLES MOBILES
    // ----------------------------------------

    struct ObstaclePrediction
    {
        std::uint32_t obstacleId = 0;
        std::vector<PredictedObstaclePoint> states;
    };

    std::vector<ObstaclePrediction> obstaclePredictions;

    const float cacheAge = localGrid.GetPredictionCacheAge();

    const auto& relevantIds =
        localGrid.GetRelevantObstacleIds();

    const auto& cachedTrajectories =
        localGrid.GetCachedObstacleTrajectories();

    // Aucun obstacle pertinent : aucune prédiction à traiter.
    if (!relevantIds.empty())
    {
        if (!std::isfinite(cacheAge) || cacheAge < 0.0f ||
            cachedTrajectories.size() != relevantIds.size())
        {
            previousVelocity = { 0.0f, 0.0f };
            return previousVelocity;
        }

        obstaclePredictions.reserve(relevantIds.size());

        for (std::size_t i = 0; i < relevantIds.size(); ++i)
        {
            const CachedObstacleTrajectory& trajectory =
                cachedTrajectories[i];

            // Vérifier la correspondance exacte des identifiants.
            if (trajectory.obstacleId != relevantIds[i] ||
                trajectory.samples.size() < 2)
            {
                previousVelocity = { 0.0f, 0.0f };
                return previousVelocity;
            }

            ObstaclePrediction prediction;
            prediction.obstacleId = trajectory.obstacleId;

            prediction.states.reserve(predictionSteps + 1);

            const auto& samples = trajectory.samples;

            // Interpolation effectuée une seule fois par instant
            // et par obstacle, avant le Beam Search.
            for (int step = 0; step <= predictionSteps; ++step)
            {
                const float plannerTime = std::min(
                    step * timeStep,
                    horizon
                );

                const float queryTime = cacheAge + plannerTime;

                if (!std::isfinite(queryTime) ||
                    queryTime < 0.0f ||
                    queryTime > samples.back().time)
                {
                    previousVelocity = { 0.0f, 0.0f };
                    return previousVelocity;
                }

                const auto upper = std::lower_bound(
                    samples.begin(),
                    samples.end(),
                    queryTime,
                    [](const PredictedObstaclePoint& sample, float time)
                    {
                        return sample.time < time;
                    }
                );

                PredictedObstaclePoint result;

                if (upper == samples.begin())
                {
                    result = *upper;
                }
                else
                {
                    if (upper == samples.end())
                    {
                        previousVelocity = { 0.0f, 0.0f };
                        return previousVelocity;
                    }

                    const auto lower = upper - 1;

                    const float interval =
                        upper->time - lower->time;

                    if (!std::isfinite(interval) ||
                        interval <= EPSILON)
                    {
                        previousVelocity = { 0.0f, 0.0f };
                        return previousVelocity;
                    }

                    const float alpha = std::clamp(
                        (queryTime - lower->time) / interval,
                        0.0f,
                        1.0f
                    );

                    result.obstacleId = trajectory.obstacleId;
                    result.time = queryTime;

                    result.position = {
                        lower->position.x +
                            (upper->position.x - lower->position.x) * alpha,

                        lower->position.y +
                            (upper->position.y - lower->position.y) * alpha,

                        lower->position.z +
                            (upper->position.z - lower->position.z) * alpha
                    };

                    result.size = {
                        lower->size.x +
                            (upper->size.x - lower->size.x) * alpha,

                        lower->size.y +
                            (upper->size.y - lower->size.y) * alpha,

                        lower->size.z +
                            (upper->size.z - lower->size.z) * alpha
                    };
                }

                prediction.states.push_back(result);
            }

            obstaclePredictions.push_back(std::move(prediction));
        }
    }

    // ----------------------------------------
    // 4. PREPARATION DU CHEMIN GLOBAL
    // ----------------------------------------

    std::vector<Vector2> routePoints;
    routePoints.reserve(globalPath.size());

    for (const GridCell& cell : globalPath)
    {
        const Vector2 position =
            globalGrid.CellToWorld(cell);

        routePoints.push_back(position);
    }

    auto GetRouteDistance = [&](
        const Vector2& position
    )
    {
        if (routePoints.empty())
            return Length({
                target.x - position.x,
                target.y - position.y
            });

        float bestDistanceSquared =
            std::numeric_limits<float>::infinity();

        for (const Vector2& point : routePoints)
        {
            const float dx = point.x - position.x;
            const float dz = point.y - position.y;

            const float distanceSquared =
                dx * dx + dz * dz;

            bestDistanceSquared = std::min(
                bestDistanceSquared,
                distanceSquared
            );
        }

        return std::sqrt(bestDistanceSquared);
    };


    // ----------------------------------------
    // 5. VERIFICATION D'UN SEGMENT
    // ----------------------------------------

    auto IsMovementSafe = [&](
        const Vector2& start,
        const Vector2& end,
        int predictionIndex,
        float& minimumClearance
    )
    {
        ++lastDiagnostics.checkedSegments;

        // Vérification statique au milieu du déplacement.
        const Vector2 middle = {
            (start.x + end.x) * 0.5f,
            (start.y + end.y) * 0.5f
        };

        if (!localGrid.IsStaticFootprintFree(
            middle.x,
            middle.y,
            0.0f
        ))
        {
            ++lastDiagnostics.rejectedSegments;
            return false;
        }

        if (!localGrid.IsStaticFootprintFree(
            end.x,
            end.y,
            0.0f
        ))
        {
            ++lastDiagnostics.rejectedSegments;
            return false;
        }

        // Vérification temporelle des obstacles mobiles.
        // Les états viennent du cache LocalGrid.

        
        for (const ObstaclePrediction& prediction :
            obstaclePredictions)
        {
            if (predictionIndex < 0 ||
                predictionIndex + 1 >=
                    static_cast<int>(prediction.states.size()))
            {
                ++lastDiagnostics.rejectedSegments;
                return false;
            }

            const PredictedObstaclePoint& obstacleStart =
                prediction.states[predictionIndex];

            const PredictedObstaclePoint& obstacleEnd =
                prediction.states[predictionIndex + 1];

            const Vector2 relativeStart = {
                start.x - obstacleStart.position.x,
                start.y - obstacleStart.position.z
            };

            const Vector2 relativeEnd = {
                end.x - obstacleEnd.position.x,
                end.y - obstacleEnd.position.z
            };

            const float halfWidth =
                std::max(
                    obstacleStart.size.x,
                    obstacleEnd.size.x
                ) * 0.5f + robotRadius;

            const float halfDepth =
                std::max(
                    obstacleStart.size.z,
                    obstacleEnd.size.z
                ) * 0.5f + robotRadius;

            if (SegmentIntersectsAABB(
                relativeStart,
                relativeEnd,
                halfWidth,
                halfDepth
            ))
            {
                ++lastDiagnostics.rejectedSegments;
                return false;
            }

            const Vector2 relativeMiddle = {
                (relativeStart.x + relativeEnd.x) * 0.5f,
                (relativeStart.y + relativeEnd.y) * 0.5f
            };

            const float clearance = DistanceToAABB(
                relativeMiddle,
                halfWidth,
                halfDepth
            );

            minimumClearance = std::min(
                minimumClearance,
                clearance
            );
        }


        return true;
    };

    // ----------------------------------------
    // 6. STRUCTURE D'UNE TRAJECTOIRE
    // ----------------------------------------

    struct TrajectoryNode
    {
        Vector2 position = { 0.0f, 0.0f };
        Vector2 velocity = { 0.0f, 0.0f };
        Vector2 firstVelocity = { 0.0f, 0.0f };

        float score = 0.0f;
        float minimumClearance =
            std::numeric_limits<float>::infinity();
    };

    std::vector<TrajectoryNode> beam;
    beam.reserve(beamWidth);

    TrajectoryNode initialNode;

    initialNode.position = robotStart;
    initialNode.velocity = previousVelocity;
    initialNode.firstVelocity = previousVelocity;

    beam.push_back(initialNode);
    TrajectoryNode bestSafePrefix;
    bool hasSafePrefix = false;
    int safePrefixSegments = 0;

    // ----------------------------------------
    // 7. DIRECTIONS CANDIDATES
    // ----------------------------------------

    constexpr std::array<float, 16> angleOffsets =
    {
        -3.1415927f,
        -2.7488936f,
        -2.3561945f,
        -1.9634954f,
        -1.5707963f,
        -1.1780972f,
        -0.7853982f,
        -0.3926991f,
        0.0f,
        0.3926991f,
        0.7853982f,
        1.1780972f,
        1.5707963f,
        1.9634954f,
        2.3561945f,
        2.7488936f
    };

    constexpr std::array<float, 2> speedFactors =
    {
        0.60f,
        1.00f
    };

    // ----------------------------------------
    // 8. RECHERCHE MULTI-SEGMENTS
    // ----------------------------------------

    for (int segment = 0;
         segment < trajectorySegments;
         ++segment)
    {
        std::vector<TrajectoryNode> nextBeam;

        nextBeam.reserve(
            beamWidth * (angleOffsets.size() * speedFactors.size() + 1)
        );

        const int firstStep =
            segment * stepsPerSegment;

        const int lastStep = std::min(
            firstStep + stepsPerSegment,
            predictionSteps
        );

        if (firstStep >= lastStep)
            break;

        for (const TrajectoryNode& node : beam)
        {
            // Candidats directionnels.
            for (float angleOffset : angleOffsets)
            {
                const float angle =
                    std::atan2(
                        desiredDirection.y,
                        desiredDirection.x
                    ) + angleOffset;

                for (float speedFactor : speedFactors)
                {
                    ++lastDiagnostics.evaluatedTrajectories;
                    TrajectoryNode candidate = node;

                    const Vector2 targetVelocity = {
                        std::cos(angle) *
                            limitedSpeed * speedFactor,

                        std::sin(angle) *
                            limitedSpeed * speedFactor
                    };

                    bool safe = true;

                    candidate.velocity = targetVelocity;

                    for (int step = firstStep;
                         step < lastStep;
                         ++step)
                    {
                        const Vector2 nextPosition = {
                            candidate.position.x +
                                candidate.velocity.x * timeStep,

                            candidate.position.y +
                                candidate.velocity.y * timeStep
                        };

                        if (!IsMovementSafe(
                            candidate.position,
                            nextPosition,
                            step,
                            candidate.minimumClearance
                        ))
                        {
                            safe = false;
                            break;
                        }

                        candidate.position = nextPosition;

                        // Pénalité de proximité des obstacles.
                        if (candidate.minimumClearance < 1.5f)
                        {
                            const float shortage =
                                1.5f -
                                candidate.minimumClearance;

                            candidate.score +=
                                shortage * shortage *
                                timeStep * 0.35f;
                        }

                        if (step == 0)
                        {
                            candidate.firstVelocity =
                                candidate.velocity;
                        }
                    }

                    if (!safe)
                        continue;

                    nextBeam.push_back(candidate);
                }
            }

            // Candidat d'arrêt / ralentissement.
            {
                ++lastDiagnostics.evaluatedTrajectories;
                TrajectoryNode candidate = node;

                bool safe = true;
                const float previousSpeed = Length(candidate.velocity);
                candidate.velocity = { 0.0f, 0.0f };
                candidate.score += previousSpeed * previousSpeed * 0.05f;

                for (int step = firstStep;
                     step < lastStep;
                     ++step)
                {
                    const Vector2 nextPosition = {
                        candidate.position.x +
                            candidate.velocity.x * timeStep,

                        candidate.position.y +
                            candidate.velocity.y * timeStep
                    };

                    if (!IsMovementSafe(
                        candidate.position,
                        nextPosition,
                        step,
                        candidate.minimumClearance
                    ))
                    {
                        safe = false;
                        break;
                    }

                    candidate.position = nextPosition;

                    if (step == 0)
                    {
                        candidate.firstVelocity =
                            candidate.velocity;
                    }
                }

                if (safe)
                {
                    nextBeam.push_back(candidate);
                }
            }
        }

        if (nextBeam.empty())
        {
            beam.clear();
            break;
        }

        // ------------------------------------
        // 9. EVALUATION ET ELAGAGE
        // ------------------------------------

        for (TrajectoryNode& candidate : nextBeam)
        {
            const float targetDistance = Length({
                target.x - candidate.position.x,
                target.y - candidate.position.y
            });

            const float routeDistance =
                GetRouteDistance(candidate.position);

            const float progress =
                initialTargetDistance - targetDistance;

            // Favoriser la progression vers la cible.
            candidate.score -= progress * 1.5f;

            // Favoriser la réinsertion sur le chemin global.
            candidate.score += routeDistance * 0.5;//2.0f;

            // Conserver une direction utile vers la cible.
            candidate.score += targetDistance * 0.35f;

            const float previousSpeed = Length(previousVelocity);
            const float candidateFirstSpeed =
                Length(candidate.firstVelocity);
            if (previousSpeed > 0.2f &&
                candidateFirstSpeed > 0.2f)
            {
                const float directionAlignment = std::clamp(
                    (previousVelocity.x * candidate.firstVelocity.x +
                     previousVelocity.y * candidate.firstVelocity.y) /
                        (previousSpeed * candidateFirstSpeed),
                    -1.0f,
                    1.0f
                );
                candidate.score +=
                    (1.0f - directionAlignment) * 0.75f;
            }

            if (preferredAvoidanceSide != 0 &&
                candidateFirstSpeed > 0.2f)
            {
                const float lateralVelocity =
                    desiredDirection.x *
                        candidate.firstVelocity.y -
                    desiredDirection.y *
                        candidate.firstVelocity.x;
                const int candidateSide =
                    lateralVelocity > candidateFirstSpeed * 0.2f
                    ? 1
                    : lateralVelocity < -candidateFirstSpeed * 0.2f
                        ? -1
                        : 0;

                if (candidateSide != 0 &&
                    candidateSide != preferredAvoidanceSide)
                {
                    candidate.score += 3.0f;
                }
            }
        }

        std::stable_sort(
            nextBeam.begin(),
            nextBeam.end(),
            [](const TrajectoryNode& a,
               const TrajectoryNode& b)
            {
                return a.score < b.score;
            }
        );

        const auto bestMovingPrefix = std::find_if(
            nextBeam.begin(),
            nextBeam.end(),
            [](const TrajectoryNode& candidate)
            {
                return Length(candidate.firstVelocity) > EPSILON;
            }
        );
        if (bestMovingPrefix != nextBeam.end())
        {
            bestSafePrefix = *bestMovingPrefix;
            hasSafePrefix = true;
            safePrefixSegments = segment + 1;
        }
        else if (!hasSafePrefix && !nextBeam.empty())
        {
            bestSafePrefix = nextBeam.front();
            safePrefixSegments = segment + 1;
        }

        if (nextBeam.size() > beamWidth)
        {
            std::vector<TrajectoryNode> diverseBeam;
            diverseBeam.reserve(beamWidth);
            std::array<bool, 8> occupiedDirectionBins = {};
            std::array<std::size_t, beamWidth> selectedIndices = {};
            std::size_t selectedCount = 0;

            for (std::size_t i = 0;
                 i < nextBeam.size() &&
                     diverseBeam.size() < beamWidth;
                 ++i)
            {
                const Vector2 direction =
                    Length(nextBeam[i].firstVelocity) > EPSILON
                    ? Normalize(nextBeam[i].firstVelocity)
                    : Normalize(nextBeam[i].velocity);

                if (Length(direction) <= EPSILON)
                    continue;

                float relativeAngle =
                    std::atan2(direction.y, direction.x) -
                    std::atan2(desiredDirection.y, desiredDirection.x);
                while (relativeAngle < 0.0f)
                    relativeAngle += 2.0f * PI_VALUE;
                while (relativeAngle >= 2.0f * PI_VALUE)
                    relativeAngle -= 2.0f * PI_VALUE;

                const std::size_t directionBin =
                    static_cast<std::size_t>(
                        relativeAngle * 8.0f / (2.0f * PI_VALUE)
                    ) % occupiedDirectionBins.size();

                if (occupiedDirectionBins[directionBin])
                    continue;

                occupiedDirectionBins[directionBin] = true;
                selectedIndices[selectedCount++] = i;
                diverseBeam.push_back(std::move(nextBeam[i]));
            }

            for (std::size_t i = 0;
                 i < nextBeam.size() &&
                     diverseBeam.size() < beamWidth;
                 ++i)
            {
                if (std::find(
                    selectedIndices.begin(),
                    selectedIndices.begin() + selectedCount,
                    i
                ) != selectedIndices.begin() + selectedCount)
                {
                    continue;
                }

                diverseBeam.push_back(std::move(nextBeam[i]));
            }

            beam = std::move(diverseBeam);
        }
        else
        {
            beam = std::move(nextBeam);
        }
    }

    // ----------------------------------------
    // 10. SELECTION DE LA MEILLEURE TRAJECTOIRE
    // ----------------------------------------

    if (beam.empty())
    {
        if (hasSafePrefix)
        {
            previousVelocity = bestSafePrefix.firstVelocity;
            lastDiagnostics.selectedSpeed = Length(previousVelocity);
            lastDiagnostics.selectedSafePrefix = true;
            lastDiagnostics.safePrefixSegments = safePrefixSegments;
            lastDiagnostics.preferredAvoidanceSide =
                preferredAvoidanceSide;
            const float resultSpeed = Length(previousVelocity);
            const float lateralVelocity =
                desiredDirection.x * previousVelocity.y -
                desiredDirection.y * previousVelocity.x;
            if (resultSpeed > 0.2f &&
                std::abs(lateralVelocity) > resultSpeed * 0.2f)
            {
                lastDiagnostics.selectedAvoidanceSide =
                    lateralVelocity > 0.0f ? 1 : -1;
            }
            if (dynamicThreatActive &&
                preferredAvoidanceSide == 0)
            {
                if (resultSpeed > 0.2f &&
                    std::abs(lateralVelocity) > resultSpeed * 0.2f)
                {
                    preferredAvoidanceSide =
                        lateralVelocity > 0.0f ? 1 : -1;
                    preferredSideHoldTime = 0.0f;
                }
            }
            return previousVelocity;
        }

        previousVelocity = { 0.0f, 0.0f };
        return previousVelocity;
    }

    const auto best = std::min_element(
        beam.begin(),
        beam.end(),
        [](const TrajectoryNode& a,
           const TrajectoryNode& b)
        {
            return a.score < b.score;
        }
    );

    Vector2 result = best->firstVelocity;

    previousVelocity = result;
    lastDiagnostics.selectedSpeed = Length(result);
    if (dynamicThreatActive &&
        preferredAvoidanceSide == 0)
    {
        const float resultSpeed = Length(result);
        const float lateralVelocity =
            desiredDirection.x * result.y -
            desiredDirection.y * result.x;
        if (resultSpeed > 0.2f &&
            std::abs(lateralVelocity) > resultSpeed * 0.2f)
        {
            preferredAvoidanceSide =
                lateralVelocity > 0.0f ? 1 : -1;
            preferredSideHoldTime = 0.0f;
        }
    }
    lastDiagnostics.preferredAvoidanceSide =
        preferredAvoidanceSide;
    const float resultSpeed = Length(result);
    const float lateralVelocity =
        desiredDirection.x * result.y -
        desiredDirection.y * result.x;
    if (resultSpeed > 0.2f &&
        std::abs(lateralVelocity) > resultSpeed * 0.2f)
    {
        lastDiagnostics.selectedAvoidanceSide =
            lateralVelocity > 0.0f ? 1 : -1;
    }

    return previousVelocity;
}
