#ifndef PROBLEM4_DIRECTIONAL_GEOMETRY_HPP
#define PROBLEM4_DIRECTIONAL_GEOMETRY_HPP

#include "runtime.hpp"
#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace directional_geometry {

struct SearchMesh {
    std::vector<Point> stations;
    std::vector<std::array<int, 3>> triangles;
    double outerRadius = 1999.0;
};

inline SearchMesh makeSearchMesh() {
    SearchMesh result;
    result.stations.push_back({0, 0});
    for (int k = 0; k < 7; ++k)
        result.stations.push_back(unit(k * 360.0 / 7) * result.outerRadius);
    for (int k = 0; k < 7; ++k)
        result.stations.push_back(result.stations[1 + k] * .5);
    for (int k = 0; k < 7; ++k)
        result.stations.push_back((result.stations[1 + k] + result.stations[1 + (k + 1) % 7]) * .5);
    for (int k = 0; k < 7; ++k) {
        const int j = (k + 1) % 7;
        const int outer = 1 + k, nextOuter = 1 + j;
        const int inner = 8 + k, nextInner = 8 + j, midpoint = 15 + k;
        result.triangles.push_back({{0, inner, nextInner}});
        result.triangles.push_back({{outer, midpoint, inner}});
        result.triangles.push_back({{inner, midpoint, nextInner}});
        result.triangles.push_back({{nextOuter, nextInner, midpoint}});
    }
    return result;
}

inline std::vector<Point> searchPoints() {
    return makeSearchMesh().stations;
}

inline double crossProduct(Point a, Point b) {
    return a.x * b.y - a.y * b.x;
}

inline bool contains(const SearchMesh& mesh, const std::array<int, 3>& triangle,
                     Point point, double tolerance = 1e-7) {
    for (int k = 0; k < 3; ++k) {
        Point a = mesh.stations.at(triangle[k]);
        Point b = mesh.stations.at(triangle[(k + 1) % 3]);
        if (crossProduct(b - a, point - a) < -tolerance) return false;
    }
    return true;
}

// The 28 faces tile the outer heptagon. Every face has diameter 999.5 m.
// If X = sum(lambda_i V_i), sum(lambda_i n.(V_i-X)) = 0; hence at least
// one face vertex lies in every closed transmitting half-plane through X.
inline bool verifySearchMesh(const SearchMesh& mesh, std::string* reason = nullptr) {
    auto fail = [&](const char* message) {
        if (reason) *reason = message;
        return false;
    };
    if (mesh.stations.size() != 22 || mesh.triangles.size() != 28)
        return fail("Expected 22 distinct stations and 28 triangles");
    for (size_t i = 0; i < mesh.stations.size(); ++i) {
        Point point = mesh.stations[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
            return fail("Nonfinite station");
        for (size_t j = 0; j < i; ++j)
            if (dist(point, mesh.stations[j]) < 1e-5) return fail("Duplicate station");
    }
    double sumArea = 0;
    std::map<std::pair<int, int>, int> edgeCounts;
    for (const auto& triangle : mesh.triangles) {
        for (int index : triangle)
            if (index < 0 || index >= static_cast<int>(mesh.stations.size()))
                return fail("Triangle index outside station array");
        Point a = mesh.stations[triangle[0]], b = mesh.stations[triangle[1]], c = mesh.stations[triangle[2]];
        double twiceArea = crossProduct(b - a, c - a);
        if (twiceArea <= 0) return fail("Triangle must be nondegenerate and counterclockwise");
        sumArea += twiceArea * .5;
        for (int k = 0; k < 3; ++k) {
            int i = triangle[k], j = triangle[(k + 1) % 3];
            if (dist(mesh.stations[i], mesh.stations[j]) > 999.501)
                return fail("Triangle edge exceeds the reception margin");
            if (i > j) std::swap(i, j);
            ++edgeCounts[{i, j}];
        }
    }
    int boundaryEdges = 0;
    for (const auto& entry : edgeCounts) {
        if (entry.second == 1) ++boundaryEdges;
        else if (entry.second != 2) return fail("Invalid mesh edge multiplicity");
    }
    if (boundaryEdges != 14) return fail("Expected 14 half-edges on the heptagon boundary");
    double polygonArea = 0;
    for (int k = 0; k < 7; ++k) {
        Point a = mesh.stations[1 + k], b = mesh.stations[1 + (k + 1) % 7];
        double edgeDistance = crossProduct(a, b) / dist(a, b);
        if (edgeDistance < 1800.5) return fail("Outer heptagon does not cover the 1800 m disk with margin");
        polygonArea += crossProduct(a, b) * .5;
    }
    if (std::abs(sumArea - polygonArea) > .001)
        return fail("Triangle areas do not sum to heptagon area");
    if (reason) reason->clear();
    return true;
}

struct BracketStep {
    Point anchor, axis, normal, plus, minus;
    double oldUpper = 0;
    double forward = 0, lateral = 0;
    double receivedUpper = 0, missedUpper = 0;
};

// Preconditions: anchor previously received this source, bearing error <= ERR,
// and the true source-anchor distance is <= upper <= 1500 m.
inline BracketStep bracketStep(Point anchor, double angle, double upper, double transverseFraction = .25) {
    if (!std::isfinite(anchor.x) || !std::isfinite(anchor.y) ||
        !std::isfinite(angle) || !std::isfinite(upper) || upper <= 0 || upper > 1500.00001)
        throw std::invalid_argument("Invalid directional bracket anchor, angle or upper bound");
    if (!std::isfinite(transverseFraction) || transverseFraction <= 0)
        throw std::invalid_argument("Invalid directional bracket transverse fraction");
    const double epsilon = ERR * PI / 180;
    const double cosine = std::cos(epsilon), sine = std::sin(epsilon);
    BracketStep step;
    step.anchor = anchor;
    step.axis = unit(angle);
    step.normal = {-step.axis.y, step.axis.x};
    step.oldUpper = upper;
    step.forward = upper / (2 * cosine);
    step.lateral = transverseFraction * upper;
    if (step.lateral <= step.forward * std::tan(epsilon))
        throw std::invalid_argument("Bearing error too large for the directional bracket");
    Point center = anchor + step.axis * step.forward;
    step.plus = center + step.normal * step.lateral;
    step.minus = center - step.normal * step.lateral;
    const double base = step.forward * step.forward + step.lateral * step.lateral;
    const double farSquared = upper * upper + base -
        2 * upper * (step.forward * cosine - step.lateral * sine);
    step.receivedUpper = std::sqrt(std::max(base, farSquared)) + 1e-6;
    step.missedUpper = step.forward / cosine + 1e-6;
    if (step.receivedUpper >= 999.9)
        throw std::invalid_argument("Directional bracket must remain within guaranteed reception range");
    return step;
}

// A positive reading at either G permits rebasing the next bracket there.
// This bound follows from the old wedge, independently of signal direction.
inline Poly afterReception(const Poly& previous, const BracketStep& step, bool plus) {
    return disk(previous, plus ? step.plus : step.minus, step.receivedUpper);
}

// Call only after both G points report no_signal. Both are within 1000 m.
// For a source beyond their transverse line, the segment A-X intersects G+G-;
// since A receives, one endpoint must also be in the closed transmitting
// half-plane. Two misses therefore put X on the A side of that line.
inline Poly afterTwoMisses(const Poly& previous, const BracketStep& step) {
    Poly next = clip(previous, step.anchor + step.axis * (step.forward + 1e-6), step.axis * -1.0);
    return disk(next, step.anchor, step.missedUpper);
}

// F points are optional parallax probes. A miss at F alone proves no range
// exclusion: it can be caused by either distance or the transmitting direction.
inline std::array<Point, 2> transverseCandidates(Point anchor, double angle, double offset) {
    if (!std::isfinite(offset) || offset < 0)
        throw std::invalid_argument("Invalid transverse offset");
    Point axis = unit(angle), normal{-axis.y, axis.x};
    return {{anchor + normal * offset, anchor - normal * offset}};
}

} // namespace directional_geometry

#endif
