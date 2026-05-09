#include "engine/nfp_solver_cache.h"

#include "core/math_utils.h"
#include <cmath>

namespace nest {

size_t NfpSolverCache::Hasher::operator()(const NfpSolverCacheKey& key) const noexcept {
    size_t h = key.movingPartId * 1469598103934665603ull;
    auto mix = [&](size_t value) {
        h ^= value + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    };
    mix(key.fixedPartId);
    mix(static_cast<size_t>(key.movingAngleBucket));
    mix(static_cast<size_t>(key.fixedAngleBucket));
    mix(key.movingMirrored ? 1u : 0u);
    mix(key.fixedMirrored ? 1u : 0u);
    mix(static_cast<size_t>(key.spacingBucket));
    mix(static_cast<size_t>(key.toleranceBucket));
    mix(static_cast<size_t>(key.geometryVersion));
    return h;
}

bool NfpSolverCache::find(const NfpSolverCacheKey& key, NfpSolverCacheValue& out) const {
    const auto it = values_.find(key);
    if (it == values_.end()) {
        ++misses_;
        return false;
    }
    ++hits_;
    out = it->second;
    return true;
}

void NfpSolverCache::store(const NfpSolverCacheKey& key, NfpSolverCacheValue value) {
    values_[key] = std::move(value);
}

void NfpSolverCache::clear() {
    values_.clear();
    hits_ = 0;
    misses_ = 0;
}

int nfpSolverAngleBucket(double radians) {
    constexpr double bucketDegrees = 0.05;
    return static_cast<int>(std::llround(radiansToDegrees(radians) / bucketDegrees));
}

int nfpSolverSpacingBucket(double spacing) {
    return static_cast<int>(std::llround(spacing * 1000.0));
}

int nfpSolverToleranceBucket(double tolerance) {
    return static_cast<int>(std::llround(tolerance * 1000000.0));
}

uint64_t nfpSolverGeometryVersion(const Part& moving, const Part& fixed, size_t movingPartId, size_t fixedPartId) {
    uint64_t h = (static_cast<uint64_t>(movingPartId) << 32u) ^ static_cast<uint64_t>(fixedPartId);
    auto mix = [&](uint64_t value) {
        h ^= value + 0x9e3779b97f4a7c15ull + (h << 6u) + (h >> 2u);
    };
    mix(static_cast<uint64_t>(moving.rings.size()));
    mix(static_cast<uint64_t>(fixed.rings.size()));
    mix(static_cast<uint64_t>(std::llround(moving.area * 1000.0)));
    mix(static_cast<uint64_t>(std::llround(fixed.area * 1000.0)));
    mix(static_cast<uint64_t>(std::llround(moving.localBounds.width() * 1000.0)));
    mix(static_cast<uint64_t>(std::llround(moving.localBounds.height() * 1000.0)));
    mix(static_cast<uint64_t>(std::llround(fixed.localBounds.width() * 1000.0)));
    mix(static_cast<uint64_t>(std::llround(fixed.localBounds.height() * 1000.0)));
    return h;
}

NfpSolverCacheValue toSolverCacheValue(const NoFitPolygonResult& result) {
    NfpSolverCacheValue value;
    value.componentCount = result.componentCount;
    value.usedDecomposition = result.usedDecomposition;
    value.loops.reserve(result.loops.size());
    for (const NoFitPolygonLoop& loop : result.loops) {
        value.loops.push_back({loop.ring, loop.bounds, loop.fromHole, loop.exactConvex});
    }
    return value;
}

NoFitPolygonResult toNoFitPolygonResult(const NfpSolverCacheValue& value) {
    NoFitPolygonResult result;
    result.componentCount = value.componentCount;
    result.usedDecomposition = value.usedDecomposition;
    result.loops.reserve(value.loops.size());
    for (const NfpSolverCachedLoop& loop : value.loops) {
        result.loops.push_back({loop.ring, loop.bounds, loop.fromHole, loop.exactConvex});
    }
    return result;
}

} // namespace nest
