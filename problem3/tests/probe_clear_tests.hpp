// Offline test code; included by tests/geometry_tests.hpp.
#ifndef PROBLEM3_PROBE_CLEAR_TESTS_HPP
#define PROBLEM3_PROBE_CLEAR_TESTS_HPP

inline void probeClearTests() {
    require(probe_clear::coverage({}, {0, 0}) == 0, "empty region cannot enable a probe clear");
    Poly square{{-50, -50}, {50, -50}, {50, 50}, {-50, 50}};
    const double diskArea = PI * 19.9 * 19.9;
    double covered = probe_clear::coverage(square, {0, 0}) * probe_clear::area(square);
    require(covered <= diskArea && covered > diskArea * .998,
            "inscribed disk coverage agrees with independent disk area");
    require(probe_clear::coverage(square, {200, 0}) == 0, "disjoint region has zero coverage");

    const Point center{200, 200};
    const Poly region{center + Point{-25, -2}, center + Point{25, -2},
                      center + Point{25, 2}, center + Point{-25, 2}};
    require(probe_clear::coverage(region, center) > .75, "narrow region permits a bounded center attempt");
    for (bool hit : {false, true}) {
        Point source = center + Point{hit ? 0.0 : 25.0, 0};
        Mock mock;
        mock.mode = 3;
        mock.sources[4] = {source, 1000};
        Session session(mock, "TEST", "");
        Planner planner(session, true);
        session.enter();
        Target& target = planner.targets[4];
        target.channel = 4;
        target.region = region;
        target.c = mec(region);
        for (Point point : {center + Point{-500, 0}, center + Point{0, -500}})
            target.obs.push_back({point, bearing(point, source)});

        planner.executeUnifiedTarget(target, center);
        require(planner.probeAttempts == 1 && target.probeAttempted,
                "pre-measure probe runs only once per target");
        if (hit) {
            require(planner.done.count(4) && target.status == "cleared" && session.measures == 0,
                    "successful probe clears without the planned measurement");
            require(session.current == 1 && session.switches == 0 && planner.probeSuccesses == 1,
                    "successful probe retains the real active channel");
            require(std::abs(session.vt - (norm(center) / 5 + 5)) < 1e-5,
                    "successful probe time contains movement and one clear");
        } else {
            require(session.failed == 1 && planner.probeFailures == 1 && session.measures == 1,
                    "failed probe immediately continues the planned real reading");
            require(target.obs.size() == 3 && target.failedClearPoints.size() == 1 &&
                    planner.negativePoints[4].empty(), "clear failure preserves bearing and reception histories");
            require(target.c.contains(source) && session.current == 4 && session.switches == 1,
                    "fallback uses the actual measurement channel and retains the real source");
            require(std::abs(session.vt - (norm(center) / 5 + 3 + 1 + 5)) < 1e-5,
                    "a failed probe adds only three seconds to the original reading");
            target.status = "detected";
            require(!planner.tryProbeClear(target, center) && planner.probeAttempts == 1,
                    "new observations do not reset the target attempt limit");
        }
        session.exit();
    }
}

#endif
