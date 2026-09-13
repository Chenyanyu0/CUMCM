// Offline test code; included by tests/geometry_tests.hpp.
#ifndef PROBLEM3_ONE_READ_TESTS_HPP
#define PROBLEM3_ONE_READ_TESTS_HPP

inline void oneReadTests() {
    const double limit = one_read_region::limit();
    const double q = 1 / (2 * std::cos(ERR * PI / 180));
    require(limit * q < 19.9 && limit > 39.79, "one-reading numerical margin");
    require(!one_read_region::contains({}, {0, 0}), "empty source region has no certificate");
    require(one_read_region::contains({{-limit, 0}, {limit, 0}}, {0, 0}), "closed certificate boundary");
    require(!one_read_region::contains({{-limit - 1e-5, 0}, {limit, 0}}, {0, 0}),
            "a vertex beyond the bound invalidates the certificate");
    require(!one_read_region::contains({{0, 0}}, {std::numeric_limits<double>::infinity(), 0}),
            "nonfinite candidates cannot be certified");

    Point center{200, -100};
    Poly region{center + Point{-30, -10}, center + Point{30, -10},
                center + Point{30, 10}, center + Point{-30, 10}};
    Circle circle = mec(region);
    vector<Point> anchors{center + Point{0, 300}, center + Point{-300, -50},
                          center + Point{300, 50}, center + Point{0, -300}};
    vector<Point> candidates = one_read_region::candidates(region, circle, anchors);
    require(candidates.size() > 1 && candidates.size() <= 8 && dist(candidates.front(), center) < 1e-7,
            "one-reading region retains center and route-dependent alternatives");
    bool usesFullRegion = false;
    for (Point candidate : candidates) {
        require(one_read_region::contains(region, candidate), "all generated points satisfy every disk");
        usesFullRegion = usesFullRegion || dist(candidate, center) > limit - circle.r + 1;
    }
    require(usesFullRegion, "use the full disk intersection rather than only the inner center disk");
    const double nearestY = std::sqrt(limit * limit - 30 * 30) - 10;
    require(std::any_of(candidates.begin(), candidates.end(), [&](Point candidate) {
        return dist(candidate, center + Point{0, nearestY}) < 1e-5;
    }), "projection agrees with the independent rectangle boundary calculation");
    Poly tooLarge{center + Point{-40, -1}, center + Point{40, -1},
                  center + Point{40, 1}, center + Point{-40, 1}};
    require(one_read_region::candidates(tooLarge, mec(tooLarge), anchors).empty(),
            "large regions retain normal refinement instead of a false one-reading certificate");

    for (int mode = 0; mode < 4; ++mode) {
        vector<Point> sources = region;
        sources.push_back(center);
        sources.push_back(center + Point{20, -4});
        for (Point source : sources) for (Point candidate : candidates) {
            Mock mock;
            mock.mode = mode;
            mock.sources[4] = {source, 1000};
            Session session(mock, "TEST", "");
            Planner planner(session, true);
            session.enter();
            Target& target = planner.targets[4];
            target.channel = 4;
            target.region = region;
            target.c = circle;
            for (Point observation : {center + Point{-500, 0}, center + Point{0, -500}})
                target.obs.push_back({observation, bearing(observation, source)});
            Target* observed = planner.observe(candidate, 4);
            require(observed && target.status != "detected" && planner.oneReadMeasures == 1 &&
                    planner.oneReadFailures == 0, "one real reading at a certified point reaches clearing precision");
            if (target.status != "cleared") {
                require(target.c.r <= 19.9 && target.c.contains(source), "certified update contains the actual source");
                planner.clearTarget(target, planner.clearPoint(target, session.pos));
            }
            require(mock.cleared.count(4) == 1 && session.failed == 0,
                    "off-center certification finishes with a successful real clear");
            session.exit();
        }
    }

    for (double halfWidth : {30.0, 100.0}) {
        Mock mock;
        Session session(mock, "TEST", "");
        Planner planner(session, true);
        Target target;
        target.channel = 7;
        target.region = {center + Point{-halfWidth, -10}, center + Point{halfWidth, -10},
                         center + Point{halfWidth, 10}, center + Point{-halfWidth, 10}};
        target.c = mec(target.region);
        for (Point observation : {center + Point{-500, 0}, center + Point{0, -500}})
            target.obs.push_back({observation, bearing(observation, center)});
        const Poly original = target.region;
        auto options = planner.taskCandidates(target, anchors);
        require(options.size() <= 10 && std::any_of(options.begin(), options.end(), [&](const Planner::CandidatePrediction& option) {
            return dist(option.point, target.c.c) < 1e-7;
        }), "candidate pruning preserves the actual MEC center for small and large regions");
        if (halfWidth == 30) {
            int certified = 0;
            for (const auto& option : options)
                if (one_read_region::contains(target.region, option.point)) ++certified;
            require(certified >= 2, "multiple one-reading positions reach the joint route window");
        }
        require(original.size() == target.region.size(), "prediction preserves the real source region");
        for (size_t i = 0; i < original.size(); ++i)
            require(dist(original[i], target.region[i]) < 1e-10, "candidate generation cannot certify a forecast region");
    }
}

#endif
