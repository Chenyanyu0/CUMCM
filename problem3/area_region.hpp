#ifndef PROBLEM3_AREA_REGION_HPP
#define PROBLEM3_AREA_REGION_HPP

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

// Include after Point.  Local coordinates use the first wedge [0, 2 degrees].
namespace area_region {

inline double radians() { return std::acos(-1.0) / 180.0; }
inline double threshold() { return 1340.63; }
inline double spacing() { return 25.0; }

namespace detail {
inline double det(Point a, Point b) { return a.x * b.y - a.y * b.x; }
inline double product(Point a, Point b) { return a.x * b.x + a.y * b.y; }
inline Point difference(Point a, Point b) { return {a.x - b.x, a.y - b.y}; }
inline double squareNorm(Point p) { return product(p, p); }
inline Point rotate(Point p, double c, double s) {
    return {p.x * c - p.y * s, p.x * s + p.y * c};
}

struct Constants {
    double sinBeta, cosBeta, sinTwiceBeta, cosTwiceBeta;
    Constants() : sinBeta(std::sin(2 * radians())),
        cosBeta(std::cos(2 * radians())),
        sinTwiceBeta(std::sin(4 * radians())),
        cosTwiceBeta(std::cos(4 * radians())) {}
};

inline const Constants& constants() {
    static const Constants value;
    return value;
}

struct HalfPlane {
    Point normal;
    double offset;
};

inline bool satisfies(Point p, const std::array<HalfPlane, 4>& planes) {
    for (const HalfPlane& plane : planes)
        if (product(plane.normal, p) < plane.offset - 1e-7) return false;
    return true;
}

inline bool directionInside(Point p, const std::array<HalfPlane, 4>& planes) {
    for (const HalfPlane& plane : planes)
        if (product(plane.normal, p) < -1e-12) return false;
    return true;
}

inline double intersectionArea(Point second, Point lower, Point upper) {
    const Constants& c = constants();
    const std::array<HalfPlane, 4> planes{{
        {{0, 1}, 0}, {{c.sinBeta, -c.cosBeta}, 0},
        {{-lower.y, lower.x}, -lower.y * second.x + lower.x * second.y},
        {{upper.y, -upper.x}, upper.y * second.x - upper.x * second.y}
    }};
    // A shared recession ray makes the unbounded-wedge model unsuitable.
    const Point rays[4] = {{1, 0}, {c.cosBeta, c.sinBeta}, lower, upper};
    for (Point ray : rays)
        if (directionInside(ray, planes)) return std::numeric_limits<double>::infinity();

    Point vertices[6];
    int count = 0;
    auto append = [&](Point p) {
        if (!satisfies(p, planes)) return;
        for (int i = 0; i < count; ++i)
            if (squareNorm(difference(p, vertices[i])) < 1e-14) return;
        vertices[count++] = p;
    };
    append({0, 0});
    append(second);
    for (int i = 0; i < 2; ++i) {
        for (int j = 2; j < 4; ++j) {
            const HalfPlane& a = planes[i];
            const HalfPlane& b = planes[j];
            double denominator = det(a.normal, b.normal);
            if (std::abs(denominator) < 1e-14) continue;
            append({(a.offset * b.normal.y - b.offset * a.normal.y) / denominator,
                (a.normal.x * b.offset - b.normal.x * a.offset) / denominator});
        }
    }
    if (count < 3) return 0;

    // Six candidates suffice for four half-planes; insertion sort avoids allocation.
    for (int i = 1; i < count; ++i) {
        Point value = vertices[i];
        int j = i;
        while (j > 0 && (vertices[j - 1].x > value.x ||
            (vertices[j - 1].x == value.x && vertices[j - 1].y > value.y))) {
            vertices[j] = vertices[j - 1];
            --j;
        }
        vertices[j] = value;
    }
    Point hull[12];
    int size = 0;
    for (int i = 0; i < count; ++i) {
        while (size >= 2 && det(difference(hull[size - 1], hull[size - 2]),
            difference(vertices[i], hull[size - 1])) <= 0) --size;
        hull[size++] = vertices[i];
    }
    const int lowerSize = size;
    for (int i = count - 2; i >= 0; --i) {
        while (size > lowerSize && det(difference(hull[size - 1], hull[size - 2]),
            difference(vertices[i], hull[size - 1])) <= 0) --size;
        hull[size++] = vertices[i];
    }
    double twiceArea = 0;
    for (int i = 1; i + 1 < size; ++i)
        twiceArea += det(difference(hull[i], hull[0]), difference(hull[i + 1], hull[0]));
    return std::abs(twiceArea) * 0.5;
}

struct ErrorRotation { double lowerCos, lowerSin, upperCos, upperSin; };
struct Quadrature {
    std::vector<Point> sources;
    std::vector<ErrorRotation> rotations;
    Quadrature(int angular, int radial, int error) {
        if (angular <= 0 || radial <= 0 || error <= 0)
            throw std::invalid_argument("Area quadrature orders must be positive");
        sources.reserve(static_cast<std::size_t>(angular) * radial);
        for (int i = 0; i < angular; ++i) {
            const double angle = 2 * radians() * (i + 0.5) / angular;
            for (int j = 0; j < radial; ++j) {
                const double radius = std::sqrt(25 + (2250000 - 25) * (j + 0.5) / radial);
                sources.push_back({radius * std::cos(angle), radius * std::sin(angle)});
            }
        }
        rotations.reserve(error);
        for (int k = 0; k < error; ++k) {
            double delta = radians() * (2 * (k + 0.5) / error - 1);
            rotations.push_back({std::cos(delta - radians()), std::sin(delta - radians()),
                std::cos(delta + radians()), std::sin(delta + radians())});
        }
    }
};

inline const Quadrature& defaultQuadrature() {
    static const Quadrature result(16, 32, 16);
    return result;
}

inline double integrate(Point second, const Quadrature& quadrature) {
    double sum = 0;
    for (Point source : quadrature.sources) {
        Point direction = difference(source, second);
        double length = std::sqrt(squareNorm(direction));
        if (!(length > 1e-12)) return std::numeric_limits<double>::infinity();
        direction.x /= length;
        direction.y /= length;
        for (const ErrorRotation& e : quadrature.rotations) {
            double area = intersectionArea(second, rotate(direction, e.lowerCos, e.lowerSin),
                rotate(direction, e.upperCos, e.upperSin));
            if (!std::isfinite(area)) return area;
            sum += area;
        }
    }
    return sum / quadrature.sources.size() / quadrature.rotations.size();
}
}  // namespace detail

inline bool finite(Point second) {
    if (!std::isfinite(second.x) || !std::isfinite(second.y)) return false;
    const detail::Constants& c = detail::constants();
    return second.x * c.sinBeta + second.y * c.cosBeta > 1500 * c.sinTwiceBeta + 1e-8 ||
        second.y * c.cosTwiceBeta - second.x * c.sinTwiceBeta < -1500 * c.sinTwiceBeta - 1e-8;
}

inline bool receive(Point second) {
    if (!std::isfinite(second.x) || !std::isfinite(second.y)) return false;
    const detail::Constants& c = detail::constants();
    return detail::squareNorm(second) <= 1000000 + 1e-7 &&
        detail::squareNorm(detail::difference(second, {1000, 0})) <= 1000000 + 1e-7 &&
        detail::squareNorm(detail::difference(second, {1000 * c.cosBeta, 1000 * c.sinBeta}))
            <= 1000000 + 1e-7;
}

inline double expectedArea(Point second, int angular = 16, int radial = 32, int error = 16) {
    if (angular <= 0 || radial <= 0 || error <= 0)
        throw std::invalid_argument("Area quadrature orders must be positive");
    if (!finite(second)) return std::numeric_limits<double>::infinity();
    if (angular == 16 && radial == 32 && error == 16)
        return detail::integrate(second, detail::defaultQuadrature());
    return detail::integrate(second, detail::Quadrature(angular, radial, error));
}

inline double observationArea(Point second, Point source, double errorDegrees) {
    if (!std::isfinite(second.x) || !std::isfinite(second.y) ||
        !std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(errorDegrees))
        throw std::invalid_argument("Area observation coordinates must be finite");
    Point direction = detail::difference(source, second);
    double length = std::sqrt(detail::squareNorm(direction));
    if (!(length > 1e-12)) return std::numeric_limits<double>::infinity();
    direction.x /= length;
    direction.y /= length;
    return detail::intersectionArea(second,
        detail::rotate(direction, std::cos((errorDegrees - 1) * radians()),
            std::sin((errorDegrees - 1) * radians())),
        detail::rotate(direction, std::cos((errorDegrees + 1) * radians()),
            std::sin((errorDegrees + 1) * radians())));
}

inline bool contains(Point second) {
    return receive(second) && finite(second) && expectedArea(second) <= threshold();
}

inline const std::vector<Point>& samples() {
    static const std::vector<Point> result = [] {
        std::vector<Point> points;
        for (int ix = 0; ix <= 40; ++ix)
            for (int iy = -36; iy <= 36; ++iy) {
                Point p(ix * spacing(), iy * spacing());
                if (contains(p)) points.push_back(p);
            }
        return points;
    }();
    return result;
}

inline Point toWorld(Point local, Point origin, double centerBearingDegrees) {
    const double angle = (centerBearingDegrees - 1) * radians();
    Point rotated = detail::rotate(local, std::cos(angle), std::sin(angle));
    return {origin.x + rotated.x, origin.y + rotated.y};
}

}  // namespace area_region
#endif
