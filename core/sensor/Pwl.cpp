#include "Pwl.h"

namespace android {

Pwl::Pwl() {}

void Pwl::clear() {
    points.clear();
}

void Pwl::addPoint(float x, float y) {
    points.push_back({x, y});
}

float Pwl::minX() const {
    return points.empty() ? 0.f : points.front().x;
}

float Pwl::maxX() const {
    return points.empty() ? 0.f : points.back().x;
}

float Pwl::eval(float x) const {
    if (points.empty()) return 0.f;
    if (points.size() == 1) return points.front().y;
    if (x <= points.front().x) return points.front().y;
    if (x >= points.back().x)  return points.back().y;

    /* Linear scan — tuning PWLs are 4-12 points; the cache cost of a
     * binary-search branch tree exceeds the savings on this
     * size. eval is on the AWB hot path but called O(1) per stats
     * tick (priors interpolation + per-CT-step lookup), not per-pixel. */
    for (size_t i = 1; i < points.size(); ++i) {
        if (x <= points[i].x) {
            const float x0 = points[i - 1].x;
            const float y0 = points[i - 1].y;
            const float x1 = points[i].x;
            const float y1 = points[i].y;
            const float t  = (x - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }
    return points.back().y;
}

} /* namespace android */
