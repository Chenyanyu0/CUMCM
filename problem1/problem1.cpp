// B problem, question 1: bearing-only localization with a +/- 1 degree error.
// Build with: g++ -std=c++14 -O2 -Wall -Wextra -pedantic problem1.cpp -o problem1.exe
// Input: n, followed by n rows of x, y, and bearing (meters and degrees).
// Bearings use east as 0 degrees and increase counterclockwise.
//
// The localization region is the intersection of the bearing wedges. The
// half-planes are intersected with a deque in O(n log n) time. The diameter of
// the resulting polygon is computed with rotating calipers in O(m) time.

#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace localization {

// Basic geometry

using Real = long double;
constexpr Real DIST_EPS = 1e-8L;
constexpr Real ROUND_EPS = 128 * std::numeric_limits<Real>::epsilon();
constexpr Real PARALLEL_EPS = 1e-14L;
const Real PI = std::acos(-1.0L);
constexpr Real ERROR_DEG = 1.0L;

struct Point {
    Real x, y;

    Point(Real x_value = 0, Real y_value = 0) : x(x_value), y(y_value) {}

    Point operator+(Point p) const {
        return Point(x + p.x, y + p.y);
    }

    Point operator-(Point p) const {
        return Point(x - p.x, y - p.y);
    }

    Point operator*(Real k) const {
        return Point(x * k, y * k);
    }

    Point operator/(Real k) const {
        return Point(x / k, y / k);
    }
};

Real dot(Point a, Point b) {
    return a.x * b.x + a.y * b.y;
}

Real cross(Point a, Point b) {
    return a.x * b.y - a.y * b.x;
}

Real norm(Point p) {
    return std::hypot(p.x, p.y);
}

Real distance(Point a, Point b) {
    return norm(a - b);
}

Real tolerance(Real scale) {
    return DIST_EPS + ROUND_EPS * std::abs(scale);
}

struct Observation {
    Point station;
    Real bearing_deg;
};

struct HalfPlane {
    Point anchor;
    Point normal;

    bool contains(Point p) const {
        const Point offset = p - anchor;
        const Real allowed_error = tolerance(norm(p) + norm(anchor));
        return dot(normal, offset) >= -allowed_error;
    }
};

// Half-plane intersection

std::vector<HalfPlane> makeHalfPlanes(const std::vector<Observation>& observations,
                                      Point origin) {
    std::vector<HalfPlane> planes;
    for (const Observation& observation : observations) {
        const Real angle = std::fmod(observation.bearing_deg, 360.0L);
        const Real lo = (angle - ERROR_DEG) * PI / 180;
        const Real hi = (angle + ERROR_DEG) * PI / 180;
        const Point station = observation.station - origin;

        planes.push_back({station, {-std::sin(lo), std::cos(lo)}});
        planes.push_back({station, {std::sin(hi), -std::cos(hi)}});
    }
    return planes;
}

bool lineIntersection(const HalfPlane& first,
                      const HalfPlane& second,
                      Point& intersection) {
    const Point direction{first.normal.y, -first.normal.x};
    const Real denominator = dot(second.normal, direction);
    if (std::abs(denominator) <= PARALLEL_EPS) {
        return false;
    }

    const Real t =
        dot(second.normal, second.anchor - first.anchor) / denominator;
    const Point point = first.anchor + direction * t;
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        throw std::runtime_error("Numerical overflow in line intersection.");
    }

    intersection = point;
    return true;
}

Point boundaryDirection(const HalfPlane& plane) {
    return {plane.normal.y, -plane.normal.x};
}

Real directionAngle(Point vector) {
    Real angle = std::atan2(vector.y, vector.x);
    if (angle < 0) angle += 2 * PI;
    return angle;
}

bool isParallel(const HalfPlane& a, const HalfPlane& b) {
    return std::abs(cross(boundaryDirection(a), boundaryDirection(b))) <= PARALLEL_EPS;
}

bool hasSameDirection(const HalfPlane& a, const HalfPlane& b) {
    return isParallel(a, b) && dot(boundaryDirection(a), boundaryDirection(b)) > 0;
}

bool hasRecessionDirection(const std::vector<HalfPlane>& planes, Point& direction) {
    std::vector<Real> angles;
    angles.reserve(planes.size());

    for (const HalfPlane& plane : planes) {
        angles.push_back(directionAngle(plane.normal));
    }
    std::sort(angles.begin(), angles.end());

    Real largest_gap = -1.0L;
    std::size_t gap_start_index = 0;
    for (std::size_t i = 0; i < angles.size(); ++i) {
        const Real next = i + 1 < angles.size()
                              ? angles[i + 1]
                              : angles[0] + 2 * PI;
        if (next - angles[i] > largest_gap) {
            largest_gap = next - angles[i];
            gap_start_index = i;
        }
    }

    if (largest_gap < PI - PARALLEL_EPS) {
        return false;
    }

    const Real arc_start =
        gap_start_index + 1 < angles.size() ? angles[gap_start_index + 1]
                                            : angles[0];
    const Real arc_end = angles[gap_start_index] + 2 * PI;
    const Real ray_angle = (arc_start + arc_end) / 2;
    direction = {std::cos(ray_angle), std::sin(ray_angle)};
    return true;
}

std::vector<HalfPlane> sortedUniqueHalfPlanes(std::vector<HalfPlane> planes) {
    std::sort(planes.begin(), planes.end(),
              [](const HalfPlane& a, const HalfPlane& b) {
                  return directionAngle(boundaryDirection(a)) <
                         directionAngle(boundaryDirection(b));
              });

    std::vector<HalfPlane> filtered;
    for (const HalfPlane& plane : planes) {
        if (filtered.empty() || !hasSameDirection(filtered.back(), plane)) {
            filtered.push_back(plane);
        } else if (!plane.contains(filtered.back().anchor)) {
            filtered.back() = plane;
        }
    }

    if (filtered.size() > 1 && hasSameDirection(filtered.front(), filtered.back())) {
        if (!filtered.back().contains(filtered.front().anchor)) {
            filtered.front() = filtered.back();
        }
        filtered.pop_back();
    }

    return filtered;
}

std::deque<HalfPlane> halfPlaneIntersection(std::vector<HalfPlane> planes) {
    const std::vector<HalfPlane> ordered = sortedUniqueHalfPlanes(std::move(planes));
    std::deque<HalfPlane> active;

    for (const HalfPlane& plane : ordered) {
        Point intersection;
        while (active.size() > 1 &&
               lineIntersection(active[active.size() - 2], active.back(), intersection) &&
               !plane.contains(intersection)) {
            active.pop_back();
        }
        while (active.size() > 1 &&
               lineIntersection(active[0], active[1], intersection) &&
               !plane.contains(intersection)) {
            active.pop_front();
        }

        if (!active.empty() && hasSameDirection(active.back(), plane)) {
            if (!plane.contains(active.back().anchor)) {
                active.back() = plane;
            }
        } else {
            active.push_back(plane);
        }
    }

    Point intersection;
    while (active.size() > 2 &&
           lineIntersection(active[active.size() - 2], active.back(), intersection) &&
           !active.front().contains(intersection)) {
        active.pop_back();
    }
    while (active.size() > 2 &&
           lineIntersection(active[0], active[1], intersection) &&
           !active.back().contains(intersection)) {
        active.pop_front();
    }

    return active;
}

bool allContain(const std::vector<HalfPlane>& planes, Point p) {
    for (const HalfPlane& h : planes) {
        if (!h.contains(p)) return false;
    }
    return true;
}

std::vector<Point> polygonFromDeque(const std::deque<HalfPlane>& active) {
    if (active.size() < 3) return {};

    std::vector<Point> vertices;
    vertices.reserve(active.size());

    for (std::size_t i = 0; i < active.size(); ++i) {
        const HalfPlane& first = active[i];
        const HalfPlane& second = active[(i + 1) % active.size()];

        Point vertex;
        if (!lineIntersection(first, second, vertex)) return {};
        vertices.push_back(vertex);
    }

    return vertices;
}

// Convex polygon diameter

std::vector<Point> convexHull(std::vector<Point> points) {
    std::sort(points.begin(), points.end(), [](Point a, Point b) {
        if (a.x != b.x) return a.x < b.x;
        return a.y < b.y;
    });

    points.erase(
        std::unique(points.begin(), points.end(), [](Point a, Point b) {
            return a.x == b.x && a.y == b.y;
        }),
        points.end());

    if (points.size() <= 2) return points;

    std::vector<Point> lower, upper;
    for (Point p : points) {
        while (lower.size() >= 2 && cross(
                   lower.back() - lower[lower.size() - 2], p - lower.back()) <= 0) {
            lower.pop_back();
        }
        lower.push_back(p);
    }

    for (auto it = points.rbegin(); it != points.rend(); ++it) {
        while (upper.size() >= 2 && cross(
                   upper.back() - upper[upper.size() - 2], *it - upper.back()) <= 0) {
            upper.pop_back();
        }
        upper.push_back(*it);
    }

    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

struct DiameterPair {
    Point first, second;
    Real length = 0;
};

DiameterPair convexPolygonDiameter(const std::vector<Point>& polygon) {
    const std::size_t m = polygon.size();
    if (m == 0) return {};
    if (m == 1) return {polygon[0], polygon[0], 0};
    if (m == 2) {
        return {polygon[0], polygon[1], distance(polygon[0], polygon[1])};
    }

    DiameterPair answer{polygon[0], polygon[0], 0};
    const auto update = [&answer](Point first, Point second) {
        const Real length = distance(first, second);
        if (length > answer.length) {
            answer = {first, second, length};
        }
    };

    const auto twiceArea = [&polygon](std::size_t i,
                                      std::size_t next_i,
                                      std::size_t j) {
        return std::abs(
            cross(polygon[next_i] - polygon[i], polygon[j] - polygon[i]));
    };

    std::size_t j = 1;
    for (std::size_t i = 0; i < m; ++i) {
        const std::size_t next_i = (i + 1) % m;

        while (true) {
            const std::size_t next_j = (j + 1) % m;
            const Real current_area = twiceArea(i, next_i, j);
            const Real next_area = twiceArea(i, next_i, next_j);
            if (next_j == i || next_area <= current_area + PARALLEL_EPS) {
                break;
            }
            j = next_j;
        }

        update(polygon[i], polygon[j]);
        update(polygon[next_i], polygon[j]);

        // A parallel opposite edge can produce two valid antipodal vertices.
        const std::size_t next_j = (j + 1) % m;
        const Real current_area = twiceArea(i, next_i, j);
        const Real next_area = twiceArea(i, next_i, next_j);
        if (next_j != i &&
            std::abs(next_area - current_area) <= PARALLEL_EPS) {
            update(polygon[i], polygon[next_j]);
            update(polygon[next_i], polygon[next_j]);
        }
    }
    return answer;
}

// Circle utilities

struct Circle {
    Point center;
    Real radius;

    Circle(Point center_value = Point(), Real radius_value = -1)
        : center(center_value), radius(radius_value) {}

    bool contains(Point p) const {
        return radius >= 0 && distance(center, p) <= radius + tolerance(radius);
    }
};

Circle diameterCircle(Point a, Point b) {
    return {a + (b - a) / 2, distance(a, b) / 2};
}

// Localization result

enum class Status { Empty, Unbounded, Bounded };

struct Result {
    Status status = Status::Empty;
    std::vector<Point> vertices;  // Counterclockwise, in original coordinates.
    Point feasible_point, recession;
    Point diameter_a, diameter_b;
    Real diameter = 0;
    Circle diameter_disk;
    bool diameter_disk_covers = false;
};

Result solve(const std::vector<Observation>& observations) {
    if (observations.empty()) {
        throw std::invalid_argument("At least one observation is required.");
    }

    for (const Observation& o : observations) {
        if (!std::isfinite(o.station.x) || !std::isfinite(o.station.y) ||
            !std::isfinite(o.bearing_deg)) {
            throw std::invalid_argument("Coordinates and bearings must be finite.");
        }
    }
    const Point origin = observations.front().station;
    const auto planes = makeHalfPlanes(observations, origin);
    Result result;
    const std::deque<HalfPlane> active = halfPlaneIntersection(planes);

    Point feasible;
    if (active.size() < 2 ||
        !lineIntersection(active[0], active[1], feasible) ||
        !allContain(planes, feasible)) {
        return result;
    }

    result.feasible_point = feasible + origin;
    Point ray;
    if (hasRecessionDirection(planes, ray)) {
        result.status = Status::Unbounded;
        result.recession = ray;
        return result;
    }

    result.status = Status::Bounded;
    std::vector<Point> vertices = polygonFromDeque(active);
    if (vertices.empty()) {
        // A mathematically bounded intersection with no area is a point.
        vertices.push_back(feasible);
    }

    const std::vector<Point> hull = convexHull(std::move(vertices));
    const DiameterPair diameter_pair = convexPolygonDiameter(hull);
    result.diameter = diameter_pair.length;

    const Point diameter_a = diameter_pair.first;
    const Point diameter_b = diameter_pair.second;
    result.diameter_disk = diameterCircle(diameter_a, diameter_b);
    result.diameter_disk_covers = true;
    for (Point p : hull) {
        if (!result.diameter_disk.contains(p)) {
            result.diameter_disk_covers = false;
        }
    }

    result.diameter_a = diameter_a + origin;
    result.diameter_b = diameter_b + origin;
    result.diameter_disk.center = result.diameter_disk.center + origin;
    for (Point p : hull) {
        result.vertices.push_back(p + origin);
    }
    return result;
}

void printPoint(const char* label, Point p) {
    std::cout << label << ' ' << p.x << ' ' << p.y << '\n';
}

void printResult(const Result& result) {
    std::cout << std::fixed << std::setprecision(10);
    if (result.status == Status::Empty) {
        std::cout << "status EMPTY\n";
        return;
    }

    if (result.status == Status::Unbounded) {
        std::cout << "status UNBOUNDED\ndiameter INF\n";
        printPoint("feasible_point", result.feasible_point);
        printPoint("recession_direction", result.recession);
        return;
    }

    std::cout << "status BOUNDED\nvertex_count " << result.vertices.size()
              << '\n';
    for (std::size_t i = 0; i < result.vertices.size(); ++i) {
        const Point& vertex = result.vertices[i];
        std::cout << "vertex " << i + 1 << ' ' << vertex.x << ' ' << vertex.y
                  << '\n';
    }
    std::cout << "diameter " << result.diameter << '\n';
    printPoint("diameter_endpoint_a", result.diameter_a);
    printPoint("diameter_endpoint_b", result.diameter_b);
    printPoint("diameter_disk_center", result.diameter_disk.center);
    std::cout << "diameter_disk_radius " << result.diameter_disk.radius << '\n'
              << "diameter_disk_covers "
              << (result.diameter_disk_covers ? "YES" : "NO") << '\n';
}

}  // namespace localization

int main() {
    try {
        int n = 0;
        if (!(std::cin >> n) || n < 1) {
            return 1;
        }

        std::vector<localization::Observation> observations;
        observations.reserve(n);
        for (int i = 0; i < n; ++i) {
            localization::Observation observation;
            if (!(std::cin >> observation.station.x >> observation.station.y >>
                  observation.bearing_deg)) {
                return 1;
            }
            observations.push_back(observation);
        }

        localization::printResult(localization::solve(observations));
        return 0;
    } catch (...) {
        return 1;
    }
}
