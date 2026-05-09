#pragma once

#include "core/aabb.h"
#include "core/part.h"
#include "geometry/no_fit_polygon.h"
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace nest {

struct NfpSolverCacheKey {
    size_t movingPartId = 0;
    size_t fixedPartId = 0;
    int movingAngleBucket = 0;
    int fixedAngleBucket = 0;
    bool movingMirrored = false;
    bool fixedMirrored = false;
    int spacingBucket = 0;
    int toleranceBucket = 0;
    uint64_t geometryVersion = 0;

    bool operator==(const NfpSolverCacheKey& other) const = default;
};

struct NfpSolverCachedLoop {
    Ring ring;
    AABB bounds;
    bool fromHole = false;
    bool exactConvex = false;
};

struct NfpSolverCacheValue {
    std::vector<NfpSolverCachedLoop> loops;
    size_t componentCount = 0;
    bool usedDecomposition = false;
};

class NfpSolverCache {
public:
    bool find(const NfpSolverCacheKey& key, NfpSolverCacheValue& out) const;
    void store(const NfpSolverCacheKey& key, NfpSolverCacheValue value);
    void clear();

    size_t hitCount() const { return hits_; }
    size_t missCount() const { return misses_; }
    size_t size() const { return values_.size(); }

private:
    struct Hasher {
        size_t operator()(const NfpSolverCacheKey& key) const noexcept;
    };

    mutable size_t hits_ = 0;
    mutable size_t misses_ = 0;
    std::unordered_map<NfpSolverCacheKey, NfpSolverCacheValue, Hasher> values_;
};

int nfpSolverAngleBucket(double radians);
int nfpSolverSpacingBucket(double spacing);
int nfpSolverToleranceBucket(double tolerance);
uint64_t nfpSolverGeometryVersion(const Part& moving, const Part& fixed, size_t movingPartId, size_t fixedPartId);

NfpSolverCacheValue toSolverCacheValue(const NoFitPolygonResult& result);
NoFitPolygonResult toNoFitPolygonResult(const NfpSolverCacheValue& value);

} // namespace nest
