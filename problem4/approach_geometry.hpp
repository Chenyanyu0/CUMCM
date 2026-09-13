#pragma once
#include "runtime.hpp"

namespace approach_geometry {

inline bool finite(Point p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

inline double signedAreaTwice(const Poly& polygon) {
    if (polygon.size() < 3) return 0;
    double area = 0;
    for (size_t i = 1; i + 1 < polygon.size(); ++i)
        area += cross(polygon[i] - polygon[0], polygon[i + 1] - polygon[0]);
    return area;
}

inline bool validPolygon(const Poly& polygon) {
    if (polygon.size() < 3) return false;
    for (Point p : polygon) if (!finite(p)) return false;
    return std::abs(signedAreaTwice(polygon)) > 1e-9;
}

inline bool contains(const Poly& polygon, Point point, double tolerance = 1e-7) {
    if (!finite(point) || !validPolygon(polygon)) return false;
    const double sign = signedAreaTwice(polygon) > 0 ? 1.0 : -1.0;
    for (size_t i = 0; i < polygon.size(); ++i) {
        Point edge = polygon[(i + 1) % polygon.size()] - polygon[i];
        double length = norm(edge);
        if (length > 1e-12 &&
            sign * cross(edge, point - polygon[i]) < -tolerance * length)
            return false;
    }
    return true;
}

inline bool baselineSide(Point a, Point b, const Poly& region, double& side) {
    if (!finite(a) || !finite(b) || !validPolygon(region)) return false;
    Point baseline = b - a;
    double length = norm(baseline);
    if (length < 1e-6) return false;
    side = cross(baseline, region[0] - a) > 0 ? 1.0 : -1.0;
    for (Point source : region)
        if (side * cross(baseline, source - a) / length <= 1e-5) return false;
    return true;
}

inline bool certified(Point point, Point a, Point b, const Poly& region,
                      double tolerance = 1e-7) {
    double side = 0;
    if (!finite(point) || !baselineSide(a, b, region, side)) return false;
    for (Point source : region) {
        const Point triangle[] = {a, b, source};
        for (int i = 0; i < 3; ++i) {
            Point edge = triangle[(i + 1) % 3] - triangle[i];
            if (side * cross(edge, point - triangle[i]) < -tolerance * norm(edge))
                return false;
        }
    }
    return true;
}

// The received disk and half-plane contain triangle(a,b,g) by convexity.
// For a fixed receiver, its triangle inequalities are affine in g, so
// intersecting source-vertex triangles certifies the entire convex region.
inline Poly safeRegion(Point a, Point b, const Poly& region) {
    double side = 0;
    if (!baselineSide(a, b, region, side)) return {};
    Poly result = {a, b, region[0]};
    if (side < 0) std::reverse(result.begin(), result.end());
    // Inset side lines by 0.1 micrometre; keep the received baseline feasible.
    const double inset = 1e-7;
    for (Point source : region) {
        const Point triangle[] = {a, b, source};
        for (int i = 0; i < 3; ++i) {
            Point edge = triangle[(i + 1) % 3] - triangle[i];
            Point inward = Point(-edge.y, edge.x) * (side / norm(edge));
            result = clip(result, triangle[i] + inward * (i == 0 ? 0 : inset), inward);
            if (result.empty()) return {};
        }
    }
    if (!validPolygon(result)) return {};
    Point interior;
    for (Point point : result) interior = interior + point;
    interior = interior * (1.0 / result.size());
    for (Point& point : result) {
        if (!certified(point, a, b, region, 0)) {
            Point inward = interior - point;
            point = point + inward * std::min(1.0, inset / norm(inward));
            if (!certified(point, a, b, region, 0)) return {};
        }
    }
    return result;
}

inline Point project(Point point, const Poly& polygon) {
    if (!finite(point) || !validPolygon(polygon))
        throw std::invalid_argument("Projection requires a finite point and nondegenerate polygon");
    if (contains(polygon, point, 0)) return point;
    Point best = polygon[0];
    double bestDistance = dist(point, best);
    for (size_t i = 0; i < polygon.size(); ++i) {
        Point start = polygon[i], edge = polygon[(i + 1) % polygon.size()] - start;
        double lengthSquared = dot(edge, edge);
        if (lengthSquared <= 1e-24) continue;
        double t = std::max(0.0, std::min(1.0, dot(point - start, edge) / lengthSquared));
        Point candidate = start + edge * t;
        double candidateDistance = dist(point, candidate);
        if (candidateDistance < bestDistance) {
            best = candidate;
            bestDistance = candidateDistance;
        }
    }
    return best;
}

} // namespace approach_geometry
