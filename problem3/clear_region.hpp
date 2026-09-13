#ifndef PROBLEM3_CLEAR_REGION_HPP
#define PROBLEM3_CLEAR_REGION_HPP

#include <algorithm>
#include <cmath>
#include <vector>

// Include after Point, Circle, Poly, and projectToDisks.
namespace clear_region {

inline double limit() { return 19.9; }

inline bool finite(Point point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

inline bool contains(const Poly& region, Point point) {
    if (region.empty() || !finite(point)) return false;
    for (Point vertex : region)
        if (!finite(vertex) || dist(point, vertex) > limit()) return false;
    return true;
}

inline bool nearest(const Poly& region, Circle circle, Point query, Point& result) {
    if (region.empty() || !finite(query)) return false;
    for (Point vertex : region) if (!finite(vertex)) return false;
    if (contains(region, query)) { result = query; return true; }
    // The projection helper accepts 1e-7 geometric error. Shrink the disks
    // before projection, then check the original 19.9 m bound strictly.
    Point projected;
    if (projectToDisks(query, region, limit() - 2e-7, projected) && contains(region, projected)) {
        result = projected;
        return true;
    }
    // A nearly tangent feasible lens can disappear under numerical shrinking.
    if (contains(region, circle.c)) { result = circle.c; return true; }
    return false;
}

inline bool onSegment(const Poly& region, Point from, Point to, Point& result) {
    if (region.empty() || !finite(from) || !finite(to)) return false;
    if (contains(region, from)) { result = from; return true; }
    Point axis = to - from;
    double lengthSquared = dot(axis, axis);
    if (lengthSquared <= 1e-20) return false;
    double lower = 0, upper = 1, radius = limit() - 2e-7;
    for (Point vertex : region) {
        if (!finite(vertex)) return false;
        Point offset = from - vertex;
        double along = dot(offset, axis) / lengthSquared;
        double perpendicularSquared = dot(offset, offset) - along * along * lengthSquared;
        double halfSquared = (radius * radius - std::max(0.0, perpendicularSquared)) / lengthSquared;
        if (halfSquared < 0) return false;
        double half = std::sqrt(halfSquared);
        lower = std::max(lower, -along - half);
        upper = std::min(upper, -along + half);
        if (lower > upper) return false;
    }
    result = from + axis * lower;
    return contains(region, result);
}

inline std::vector<Point> candidates(const Poly& region, Circle circle,
                                     const std::vector<Point>& anchors) {
    std::vector<Point> pool, result;
    if (!contains(region, circle.c)) return result;
    auto append = [&](std::vector<Point>& points, Point point) {
        if (!contains(region, point)) return;
        for (Point old : points) if (dist(old, point) <= 1e-5) return;
        points.push_back(point);
    };
    append(pool, circle.c);
    const size_t anchorCount = std::min<size_t>(anchors.size(), 8);
    auto project = [&](Point query) {
        Point point;
        if (nearest(region, circle, query, point)) append(pool, point);
    };
    for (size_t i = 0; i < anchorCount; ++i) project(anchors[i]);
    for (size_t i = 1; i < anchorCount; ++i) {
        Point point;
        if (onSegment(region, anchors[0], anchors[i], point)) append(pool, point);
        Point axis = anchors[i] - anchors[0];
        double lengthSquared = dot(axis, axis);
        if (lengthSquared <= 1e-12) continue;
        double fraction = dot(circle.c - anchors[0], axis) / lengthSquared;
        project(anchors[0] + axis * std::max(0.0, std::min(1.0, fraction)));
        project((anchors[0] + anchors[i]) * .5);
    }

    append(result, circle.c);
    if (anchorCount && finite(anchors[0])) {
        auto nearestPoint = std::min_element(pool.begin(), pool.end(), [&](Point a, Point b) {
            return dist(anchors[0], a) < dist(anchors[0], b);
        });
        append(result, *nearestPoint);
        for (size_t i = 1; i < anchorCount && result.size() < 10; ++i) {
            if (!finite(anchors[i])) continue;
            auto best = std::min_element(pool.begin(), pool.end(), [&](Point a, Point b) {
                return dist(anchors[0], a) + dist(a, anchors[i]) <
                       dist(anchors[0], b) + dist(b, anchors[i]);
            });
            append(result, *best);
        }
    }
    for (Point point : pool) {
        if (result.size() >= 10) break;
        append(result, point);
    }
    return result;
}

} // namespace clear_region

#endif
