#pragma once

#include "core/aabb.h"
#include "core/polygon.h"
#include "core/transform.h"
#include <vector>

namespace nest {

AABB boundsOfRing(const Ring& ring);
Ring transformedRing(const Ring& ring, const Transform& transform);
std::vector<Ring> transformedRings(const std::vector<Ring>& rings, const Transform& transform);
void classifyHoleRings(std::vector<Ring>& rings);
double polygonArea(const std::vector<Ring>& rings);

} // namespace nest
