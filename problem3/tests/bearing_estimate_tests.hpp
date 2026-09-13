// Offline test code; included by tests/geometry_tests.hpp.
#ifndef PROBLEM3_BEARING_ESTIMATE_TESTS_HPP
#define PROBLEM3_BEARING_ESTIMATE_TESTS_HPP

inline void bearingEstimateTests() {
    Target target;
    target.region = {{-100, -80}, {100, -80}, {100, 80}, {-100, 80}};
    target.c = mec(target.region);
    Point source{37, -12};
    for (Point station : {Point{-500, 0}, Point{0, -600}, Point{700, 400}, Point{400, -12}})
        target.obs.push_back({station, bearing(station, source)});
    // Exercise equivalent bearings across the zero-degree boundary.
    target.obs[0].second -= 360;
    target.obs[1].second += 720;
    const Poly original = target.region;
    require(dist(bearing_estimate::estimate(target), source) < 1e-5,
            "all-bearing estimate recovers a known noiseless source across angle wrapping");
    require(target.region.size() == original.size(), "point estimation preserves the real region");
    for (size_t i = 0; i < original.size(); ++i)
        require(dist(target.region[i], original[i]) == 0, "point estimation cannot alter region vertices");

    Target constrained = target;
    constrained.region = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    constrained.c = mec(constrained.region);
    require(dist(bearing_estimate::estimate(constrained), {10, 0}) < 1e-5,
            "an inconsistent unconstrained estimate is projected onto the actual polygon");
    std::reverse(constrained.region.begin(), constrained.region.end());
    require(dist(bearing_estimate::estimate(constrained), {10, 0}) < 1e-5,
            "convex projection accepts clockwise polygon ordering");
    require(dist(bearing_estimate::project({{0, 0}, {10, 0}, {20, 0}}, {30, 0}), {20, 0}) < 1e-8,
            "collinear polygon projection does not accept points beyond the segment");

    Target parallel = target;
    parallel.obs = {{{-400, 0}, 0}, {{-600, 10}, 0}, {{-800, -10}, 180}};
    require(dist(bearing_estimate::estimate(parallel), parallel.c.c) < 1e-8,
            "parallel bearings retain the center instead of an unstable intersection");
    parallel.obs[1].second = 1e-8;
    require(dist(bearing_estimate::estimate(parallel), parallel.c.c) < 1e-8,
            "nearly parallel bearings reject ill-conditioned normal equations");
    parallel.obs.resize(1);
    require(dist(bearing_estimate::estimate(parallel), parallel.c.c) < 1e-8,
            "a single bearing cannot create a statistical range estimate");

    for (double bad : {1e308, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
        Target malformed = target;
        malformed.obs[0].first.x = bad;
        Point result = bearing_estimate::estimate(malformed);
        require(bearing_estimate::finite(result) && dist(result, target.c.c) < 1e-8,
                "malformed observation coordinates safely return the center");
        malformed = target;
        malformed.obs[0].second = bad;
        result = bearing_estimate::estimate(malformed);
        require(bearing_estimate::finite(result) && dist(result, target.c.c) < 1e-8,
                "malformed angles do not produce nonfinite estimates");
        malformed = target;
        malformed.region[0].y = bad;
        result = bearing_estimate::estimate(malformed);
        require(bearing_estimate::finite(result), "malformed region vertices cannot overflow projection");
    }
}

#endif
