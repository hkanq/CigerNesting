#pragma once

#include "core/vec2.h"
#include "engine/analytic_contact_candidate.h"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace nest {

struct NfpCacheKey {
    size_t movingPartId = 0;
    size_t fixedPartId = 0;
    int movingAngleBucket = 0;
    int fixedAngleBucket = 0;
    bool movingMirrored = false;
    bool fixedMirrored = false;
    int spacingBucket = 0;
    uint64_t geometryVersion = 0;

    bool operator==(const NfpCacheKey& other) const = default;
};

struct NfpCachedCandidate {
    Pose localPose;
    AnalyticContactKind kind = AnalyticContactKind::NfpPartPart;
    size_t sourcePart = static_cast<size_t>(-1);
    int sourceRing = -1;
    double priority = 0.0;
};

struct NfpCacheValue {
    std::vector<NfpCachedCandidate> candidates;
};

class NfpCache {
public:
    bool find(const NfpCacheKey& key, NfpCacheValue& out) const;
    void store(const NfpCacheKey& key, NfpCacheValue value);
    void clear();
    size_t hitCount() const { return hits_; }
    size_t missCount() const { return misses_; }
    size_t size() const { return values_.size(); }

private:
    struct Hasher {
        size_t operator()(const NfpCacheKey& key) const noexcept;
    };

    mutable size_t hits_ = 0;
    mutable size_t misses_ = 0;
    std::unordered_map<NfpCacheKey, NfpCacheValue, Hasher> values_;
};

int nfpAngleBucket(double radians);
int nfpSpacingBucket(double spacing);
uint64_t nfpGeometryVersion(size_t movingPartId, size_t fixedPartId);

} // namespace nest
