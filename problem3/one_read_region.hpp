#ifndef PROBLEM3_ONE_READ_REGION_HPP
#define PROBLEM3_ONE_READ_REGION_HPP

#include <algorithm>
#include <cmath>
#include <vector>

// Include after Point, Circle, Poly, and projectToDisks.
namespace one_read_region {

inline double limit() {
    return 2 * std::cos(ERR * PI / 180) * (19.9 - 1e-4);
}

inline bool contains(const Poly& region, Point point) {
    if (region.empty() || !std::isfinite(point.x) || !std::isfinite(point.y)) return false;
    const double radius = limit();
    for (Point vertex : region)
        if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) ||
            dist(point, vertex) > radius) return false;
    return true;
}

inline std::vector<Point> candidates(const Poly& region, Circle circle,
                                     const std::vector<Point>& anchors) {
    std::vector<Point> pool, result;
    if (region.empty()) return result;
    auto append = [&](std::vector<Point>& points, Point point) {
        if (!contains(region, point)) return;
        for (Point old : points) if (dist(old, point) <= 1e-5) return;
        points.push_back(point);
    };
    append(pool, circle.c);
    // For a true minimum enclosing circle, failure here means the lens is empty.
    if (pool.empty()) return result;

    auto project = [&](Point query) {
        Point point;
        // projectToDisks accepts a 1e-7 tolerance; offset it before the strict check.
        if (projectToDisks(query, region, limit() - 2e-7, point)) append(pool, point);
    };
    const size_t anchorCount = std::min<size_t>(anchors.size(), 8);
    for (size_t i = 0; i < anchorCount; ++i) project(anchors[i]);
    if (anchorCount > 1) {
        for (size_t i = 1; i < anchorCount; ++i) {
            Point axis = anchors[i] - anchors[0];
            double lengthSquared = dot(axis, axis);
            if (lengthSquared <= 1e-12) continue;
            double fraction = dot(circle.c - anchors[0], axis) / lengthSquared;
            fraction = std::max(0.0, std::min(1.0, fraction));
            project(anchors[0] + axis * fraction);
            project(anchors[0] + axis * .5);
        }
    }

    // Keep the center and nearest current-position choice, then preserve cheap
    // insertions toward different successors instead of fixing one representative.
    append(result, circle.c);
    if (anchorCount) {
        auto nearest = std::min_element(pool.begin(), pool.end(), [&](Point a, Point b) {
            return dist(anchors[0], a) < dist(anchors[0], b);
        });
        append(result, *nearest);
        for (size_t i = 1; i < anchorCount && result.size() < 8; ++i) {
            auto best = std::min_element(pool.begin(), pool.end(), [&](Point a, Point b) {
                return dist(anchors[0], a) + dist(a, anchors[i]) <
                       dist(anchors[0], b) + dist(b, anchors[i]);
            });
            append(result, *best);
        }
    }
    for (Point point : pool) {
        if (result.size() >= 8) break;
        append(result, point);
    }
    return result;
}

} // namespace one_read_region

#endif
