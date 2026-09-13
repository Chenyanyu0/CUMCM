#ifndef PROBLEM3_BEARING_ESTIMATE_HPP
#define PROBLEM3_BEARING_ESTIMATE_HPP

#include <algorithm>
#include <cmath>
#include <limits>

// Include after Point, Poly, and Target. An estimate never certifies a clear.
namespace bearing_estimate {

inline bool finite(Point point) {
    // Coordinates in this task are a few kilometres; this generous bound also
    // keeps malformed finite inputs from overflowing the normal equations.
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::abs(point.x) <= 1e9 && std::abs(point.y) <= 1e9;
}

inline Point segmentProjection(Point query, Point a, Point b) {
    Point axis = b - a;
    double lengthSquared = dot(axis, axis);
    if (lengthSquared <= 1e-20) return a;
    double fraction = dot(query - a, axis) / lengthSquared;
    return a + axis * std::max(0.0, std::min(1.0, fraction));
}

inline Point project(const Poly& region, Point query) {
    if (region.empty()) return query;
    if (region.size() == 1) return region.front();
    Point nearest = region.front();
    double nearestDistance = std::numeric_limits<double>::infinity();
    bool positive = false, negative = false;
    for (size_t i = 0; i < region.size(); ++i) {
        Point a = region[i], b = region[(i + 1) % region.size()];
        Point candidate = segmentProjection(query, a, b);
        double distance = dist(query, candidate);
        if (distance < nearestDistance) { nearest = candidate; nearestDistance = distance; }
        double side = cross(b - a, query - a);
        double tolerance = 1e-8 * norm(b - a);
        positive = positive || side > tolerance;
        negative = negative || side < -tolerance;
    }
    if (region.size() >= 3 && positive != negative) return query;
    if (nearestDistance <= 1e-8) return query;
    return nearest;
}

inline Point estimate(const Target& target) {
    Point fallback = finite(target.c.c) ? target.c.c : Point{};
    bool validRegion = !target.region.empty();
    for (Point vertex : target.region) validRegion = validRegion && finite(vertex);
    if (!validRegion) return fallback;
    fallback = project(target.region, fallback);
    if (target.obs.size() < 2) return fallback;
    for (const auto& observation : target.obs)
        if (!finite(observation.first) || !std::isfinite(observation.second) ||
            std::abs(observation.second) > 1e9) return fallback;

    Point current = fallback;
    for (int iteration = 0; iteration < 8; ++iteration) {
        double xx = 0, xy = 0, yy = 0, bx = 0, by = 0;
        for (const auto& observation : target.obs) {
            Point direction = unit(std::remainder(observation.second, 360.0));
            Point normal{-direction.y, direction.x};
            double range = std::max(20.0, dist(current, observation.first));
            double weight = 1 / (range * range);
            double offset = dot(normal, observation.first - fallback);
            xx += weight * normal.x * normal.x;
            xy += weight * normal.x * normal.y;
            yy += weight * normal.y * normal.y;
            bx += weight * normal.x * offset;
            by += weight * normal.y * offset;
        }
        double trace = xx + yy, determinant = xx * yy - xy * xy;
        if (!std::isfinite(determinant) || trace <= 0 ||
            determinant <= 1e-10 * trace * trace) return fallback;
        Point next = fallback + Point{(yy * bx - xy * by) / determinant,
                                      (xx * by - xy * bx) / determinant};
        if (!finite(next)) return fallback;
        next = project(target.region, next);
        if (dist(next, current) <= 1e-7) return next;
        current = next;
    }
    return current;
}

} // namespace bearing_estimate

#endif
