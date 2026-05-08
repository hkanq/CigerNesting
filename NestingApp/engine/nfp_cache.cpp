#include "engine/nfp_cache.h"

#include "core/math_utils.h"
#include <cmath>

namespace nest {

size_t NfpCache::Hasher::operator()(const NfpCacheKey& key) const noexcept {
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
    mix(static_cast<size_t>(key.geometryVersion));
    return h;
}

bool NfpCache::find(const NfpCacheKey& key, NfpCacheValue& out) const {
    const auto it = values_.find(key);
    if (it == values_.end()) {
        ++misses_;
        return false;
    }
    ++hits_;
    out = it->second;
    return true;
}

void NfpCache::store(const NfpCacheKey& key, NfpCacheValue value) {
    values_[key] = std::move(value);
}

void NfpCache::clear() {
    values_.clear();
    hits_ = 0;
    misses_ = 0;
}

int nfpAngleBucket(double radians) {
    constexpr double bucketDegrees = 0.05;
    return static_cast<int>(std::llround(radiansToDegrees(radians) / bucketDegrees));
}

int nfpSpacingBucket(double spacing) {
    return static_cast<int>(std::llround(spacing * 1000.0));
}

uint64_t nfpGeometryVersion(size_t movingPartId, size_t fixedPartId) {
    return (static_cast<uint64_t>(movingPartId) << 32u) ^ static_cast<uint64_t>(fixedPartId);
}

} // namespace nest
