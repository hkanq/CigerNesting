#include "geometry/transformed_shape.h"

namespace nest {

TransformedPart transformPart(const Part& part, const Pose& pose, int partId) {
    const Transform transform = pose.toTransform();
    TransformedPart out;
    out.partId = partId;
    out.rings.reserve(part.rings.size());

    for (const Ring& ring : part.rings) {
        TransformedRing transformed;
        transformed.isHole = ring.isHole;
        transformed.points.reserve(ring.points.size());
        for (const Vec2& point : ring.points) {
            const Vec2 world = transform.apply(point);
            transformed.points.push_back(world);
            transformed.bounds.include(world);
            out.bounds.include(world);
        }
        out.rings.push_back(std::move(transformed));
    }

    return out;
}

} // namespace nest
