// Offline test code; included by tests/geometry_tests.hpp.
#ifndef PROBLEM3_CLEAR_REGION_TESTS_HPP
#define PROBLEM3_CLEAR_REGION_TESTS_HPP

inline void clearRegionTests() {
    require(!clear_region::contains({}, {0, 0}), "empty polygon cannot certify a clear");
    require(clear_region::contains({{-19.9, 0}, {19.9, 0}}, {0, 0}),
            "tangent clearing disks retain their certified boundary");
    require(!clear_region::contains({{-19.90001, 0}, {19.9, 0}}, {0, 0}),
            "a distant vertex invalidates clearing certification");
    require(!clear_region::contains({{0, 0}}, {std::numeric_limits<double>::infinity(), 0}),
            "nonfinite clearing points are rejected");
    Point point;
    Poly tangent{{-19.9, 0}, {19.9, 0}};
    require(clear_region::nearest(tangent, mec(tangent), {0, 100}, point) && norm(point) < 1e-8,
            "numerical shrinking cannot discard a tangent certified lens");
    Poly emptyLens{{-20, 0}, {20, 0}};
    require(!clear_region::nearest(emptyLens, mec(emptyLens), {0, 100}, point) &&
            clear_region::candidates(emptyLens, mec(emptyLens), {{0, 100}}).empty(),
            "an infeasible clear region produces no candidate");

    Point center{200, -100};
    Poly region{center + Point{-18, -1}, center + Point{18, -1},
                center + Point{18, 1}, center + Point{-18, 1}};
    Circle circle = mec(region);
    vector<Point> anchors{center + Point{0, 100}, center + Point{0, -100},
                          center + Point{100, 20}, center + Point{-100, -20}};
    auto candidates = clear_region::candidates(region, circle, anchors);
    require(candidates.size() > 1 && candidates.size() <= 10,
            "full clear lens retains a bounded set of route alternatives");
    require(dist(candidates.front(), center) < 1e-8, "certified center remains a route alternative");
    require(clear_region::nearest(region, circle, anchors[0], point), "skinny polygon has a clear projection");
    double expectedY = std::sqrt(19.9 * 19.9 - 18 * 18) - 1;
    require(dist(point, center + Point{0, expectedY}) < 1e-5,
            "full clear projection matches the independent rectangle boundary");
    require(dist(point, center) > 19.9 - circle.r + 5,
            "full clear lens extends materially beyond the inner MEC disk");
    Point oldPoint = center + Point{0, 19.9 - circle.r};
    require(dist(anchors[0], point) + 5 < dist(anchors[0], oldPoint),
            "full clear projection reduces travel for a skinny source region");
    require(clear_region::onSegment(region, anchors[0], anchors[1], point),
            "an intersecting continuation segment supplies a zero-detour clear");
    require(std::abs(dist(anchors[0], point) + dist(point, anchors[1]) -
                     dist(anchors[0], anchors[1])) < 1e-8,
            "segment clear has zero extra route length");
    require(!clear_region::onSegment(region, center + Point{100, 100}, center + Point{-100, 100}, point),
            "a disjoint route segment has no certified stopping point");
    require(clear_region::nearest(region, circle, center, point) && dist(point, center) < 1e-8,
            "already feasible positions do not move");

    for (Point candidate : candidates) {
        require(clear_region::contains(region, candidate), "every clear alternative contains the whole source polygon");
        vector<Point> sources = region;
        sources.push_back(center);
        sources.push_back(center + Point{10, -.5});
        for (Point source : sources) {
            Mock mock;
            mock.sources[4] = {source, 1000};
            Session session(mock, "TEST", "");
            session.enter();
            require(has(session.clear(candidate, 4), "clear_result", "success") && session.failed == 0,
                    "all clear lens options clear every tested feasible source in the Mock");
            session.exit();
        }
    }
}

#endif
