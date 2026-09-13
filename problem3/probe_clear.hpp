#ifndef PROBLEM3_PROBE_CLEAR_HPP
#define PROBLEM3_PROBE_CLEAR_HPP

// Include after Poly, clip, and the geometry primitives.
namespace probe_clear {

inline double area(const Poly& polygon) {
    if (polygon.size() < 3) return 0;
    double twice = 0;
    for (size_t i = 1; i + 1 < polygon.size(); ++i)
        twice += cross(polygon[i] - polygon[0], polygon[i + 1] - polygon[0]);
    return std::abs(twice) * .5;
}

// This is an area-coverage score, not a calibrated source probability.
// The inscribed polygon slightly underestimates the 19.9 m disk coverage.
inline double coverage(const Poly& region, Point point) {
    const double total = area(region);
    if (total < 1e-6 || !std::isfinite(total) ||
        !std::isfinite(point.x) || !std::isfinite(point.y)) return 0;
    Poly inside = region;
    const int sides = 64;
    const double apothem = 19.9 * std::cos(PI / sides);
    for (int i = 0; i < sides && !inside.empty(); ++i) {
        Point normal = unit(i * 360.0 / sides);
        inside = clip(inside, point + normal * apothem, normal * -1);
    }
    return std::max(0.0, std::min(1.0, area(inside) / total));
}

} // namespace probe_clear

#endif
