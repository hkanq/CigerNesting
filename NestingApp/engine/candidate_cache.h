#pragma once

#include "core/polygon.h"
#include <cstddef>
#include <cstdint>
#include <list>
#include <unordered_map>

namespace nest {

struct CachedCandidateScore {
    double totalScore = 0.0;
    bool valid = false;
    int collisionCount = 0;
    int invalidPartCount = 0;
    double spacingPenalty = 0.0;
    double sheetPenalty = 0.0;
};

class CandidateCache {
public:
    explicit CandidateCache(size_t capacity = 4096) : capacity_(capacity) {}

    bool get(size_t partIndex, const Pose& base, const Pose& candidate, CachedCandidateScore& out) {
        const Key key = makeKey(partIndex, base, candidate);
        auto it = entries_.find(key);
        if (it == entries_.end()) {
            return false;
        }
        order_.splice(order_.begin(), order_, it->second.orderIt);
        out = it->second.score;
        return true;
    }

    void put(size_t partIndex, const Pose& base, const Pose& candidate, const CachedCandidateScore& score) {
        if (capacity_ == 0) {
            return;
        }
        const Key key = makeKey(partIndex, base, candidate);
        auto it = entries_.find(key);
        if (it != entries_.end()) {
            it->second.score = score;
            order_.splice(order_.begin(), order_, it->second.orderIt);
            return;
        }

        order_.push_front(key);
        entries_.emplace(key, Entry{score, order_.begin()});
        while (entries_.size() > capacity_) {
            const Key old = order_.back();
            order_.pop_back();
            entries_.erase(old);
        }
    }

private:
    struct Key {
        size_t partIndex = 0;
        int64_t baseX = 0;
        int64_t baseY = 0;
        int64_t baseAngle = 0;
        int64_t candidateX = 0;
        int64_t candidateY = 0;
        int64_t candidateAngle = 0;
        bool baseMirror = false;
        bool candidateMirror = false;

        bool operator==(const Key& other) const {
            return partIndex == other.partIndex &&
                baseX == other.baseX &&
                baseY == other.baseY &&
                baseAngle == other.baseAngle &&
                candidateX == other.candidateX &&
                candidateY == other.candidateY &&
                candidateAngle == other.candidateAngle &&
                baseMirror == other.baseMirror &&
                candidateMirror == other.candidateMirror;
        }
    };

    struct KeyHash {
        size_t operator()(const Key& key) const {
            size_t h = key.partIndex + 0x9e3779b97f4a7c15ull;
            auto mix = [&](uint64_t value) {
                h ^= static_cast<size_t>(value + 0x9e3779b97f4a7c15ull + (static_cast<uint64_t>(h) << 6) + (static_cast<uint64_t>(h) >> 2));
            };
            mix(static_cast<uint64_t>(key.baseX));
            mix(static_cast<uint64_t>(key.baseY));
            mix(static_cast<uint64_t>(key.baseAngle));
            mix(static_cast<uint64_t>(key.candidateX));
            mix(static_cast<uint64_t>(key.candidateY));
            mix(static_cast<uint64_t>(key.candidateAngle));
            mix(key.baseMirror ? 1u : 0u);
            mix(key.candidateMirror ? 1u : 0u);
            return h;
        }
    };

    struct Entry {
        CachedCandidateScore score;
        std::list<Key>::iterator orderIt;
    };

    static int64_t quantize(double value, double scale) {
        return static_cast<int64_t>(value * scale + (value >= 0.0 ? 0.5 : -0.5));
    }

    static Key makeKey(size_t partIndex, const Pose& base, const Pose& candidate) {
        return {
            partIndex,
            quantize(base.x, 1000.0),
            quantize(base.y, 1000.0),
            quantize(base.angleRadians, 100000.0),
            quantize(candidate.x, 1000.0),
            quantize(candidate.y, 1000.0),
            quantize(candidate.angleRadians, 100000.0),
            base.mirrored,
            candidate.mirrored
        };
    }

    size_t capacity_ = 4096;
    std::list<Key> order_;
    std::unordered_map<Key, Entry, KeyHash> entries_;
};

} // namespace nest
