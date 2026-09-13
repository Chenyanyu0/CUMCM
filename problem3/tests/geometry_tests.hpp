#ifndef PROBLEM3_TESTS_GEOMETRY_TESTS_HPP
#define PROBLEM3_TESTS_GEOMETRY_TESTS_HPP

#include "unified_tests.hpp"
#include "one_read_tests.hpp"
#include "clear_region_tests.hpp"
#include "probe_clear_tests.hpp"
#include "bearing_estimate_tests.hpp"
#include "neighborhood_route_tests.hpp"

void geometryTests() {
    const Json partial = completionMetrics(3, 12, 120);
    require(partial.at("cleared_ratio") == .25 && partial.at("average_time_s") == 40,
            "completion metrics use successful clears and the full source count");
    const Json unknown = completionMetrics(0, -1, 120);
    require(unknown.at("total_sources").is_null() && unknown.at("cleared_ratio").is_null() &&
            unknown.at("average_time_s").is_null(), "incomplete discovery and zero clears do not invent ratios");
    neighborhoodRouteTests();
    unifiedPlannerTests();
    oneReadTests();
    clearRegionTests();
    probeClearTests();
    bearingEstimateTests();
    require(std::abs(area_region::observationArea({500, 500}, unit(1) * 1000, 0) - 1199.46867746) < 1e-5,
            "problem 2 exact intersection area");
    require(!area_region::finite({500, 0}) && !area_region::contains({500, 0}), "exclude unbounded middle band");
    require(area_region::contains({960, -280}), "area valley belongs to the third contour");
    require(std::abs(area_region::expectedArea({750, -500}) -
                     area_region::expectedArea({750, -500}, 24, 64, 24)) < 2,
            "area quadrature convergence at a representative point");
    int upperBranch = 0, lowerBranch = 0;
    for (Point local : area_region::samples()) {
        if (cross(unit(1), local) > 0) ++upperBranch; else ++lowerBranch;
    }
    require(upperBranch > 0 && lowerBranch > 0, "retain both disconnected area branches");
    {
        Mock mock;
        mock.sources[7] = {{900, 100}, 1000};
        Session session(mock, "TEST", "");
        Planner planner(session, true);
        session.enter();
        require(planner.observe({0, 0}, 19) == nullptr && planner.targets.empty(),
                "no signal creates no area task");
        Target* target = planner.observe({0, 0}, 7);
        require(target && target->obs.size() == 1 && !target->secondRegion.empty() && planner.areaCreated == 1,
                "first bearing creates exactly one area task");
        Point second = target->secondRegion.front();
        require(planner.observe(second, 7) != nullptr && target->secondRegion.empty() &&
                planner.areaRetired == 1 && planner.areaMeasures == 1,
                "actual second bearing consumes the area task");
        session.exit();
    }
    {
        Mock mock;
        mock.sources[2] = {{0, 0}, 1000};
        Session session(mock, "TEST", "");
        Planner planner(session, true);
        session.enter();
        Target* target = planner.observe({0, 0}, 2);
        require(target && target->status == "cleared" && target->secondRegion.empty() && planner.areaCreated == 0,
                "near observation clears without creating an area task");
        session.exit();
    }
    Mock receptionMock;
    Session receptionSession(receptionMock, "TEST", "");
    Planner receptionPlanner(receptionSession, true);
    for (double angle : {0.0, 91.0, 180.0, 359.99}) {
        Point origin{350, -250};
        Target target;
        target.channel = 6;
        target.obs.push_back({origin, angle});
        target.region = disk(wedge(outerDisk(origin, 1500), origin, angle), origin, 1500);
        target.c = mec(target.region);
        Point transformed = area_region::toWorld({1000, 0}, origin, angle);
        require(dist(transformed, origin + unit(angle - 1) * 1000) < 1e-8,
                "area template uses the lower boundary rather than center bearing");
        for (Point local : area_region::samples()) {
            Point candidate = area_region::toWorld(local, origin, angle);
            if (!receptionPlanner.guaranteedReception(target, candidate)) continue;
            for (double offset : {-ERR, 0.0, ERR})
                for (double range : {5.0, 1000.0, 1500.0})
                    require(dist(candidate, origin + unit(angle + offset) * range) <= std::max(1000.0, range),
                            "rotated area candidates preserve reception guarantee");
        }
        vector<Point> centers = receptionPlanner.receptionLens(target.obs.front());
        for (double heading : {-175.0, -80.0, -35.0, 0.0, 35.0, 80.0, 175.0}) {
            Point projected;
            Point query = origin + unit(angle + heading) * 1100;
            require(projectToDisks(query, centers, 999.8, projected), "nonempty reception lens projection");
            require(inDisks(projected, centers, 999.8), "projected point belongs to all disks");
            require(receptionPlanner.guaranteedReception(target, projected), "lens certificate is accepted");
            for (int index = 0; index <= 40; ++index) {
                double offset = ERR * (index / 20.0 - 1);
                for (double range : {0.0, 5.0, 250.0, 999.9, 1000.0, 1250.0, 1500.0}) {
                    Point source = origin + unit(angle + offset) * range;
                    require(dist(projected, source) <= std::max(1000.0, range) + 1e-6,
                            "lens receives angular interior and near/far sources");
                }
            }
        }
        Point lensPoint = origin + unit(angle) * 100;
        require(receptionPlanner.guaranteedReception(target, lensPoint) &&
                receptionPlanner.farthest(target, lensPoint) > 1000,
                "positive reception history extends the old 1000 m test");
        Point source = origin + unit(angle) * 1400;
        require(dist(source, lensPoint) <= receptionPlanner.receptionLowerBound(target, source),
                "prediction uses the successful-observation radius lower bound");
    }
    Point projected;
    require(!projectToDisks({0, 0}, {{0, 0}, {3, 0}}, 1, projected), "disjoint disks have no projection");
    require(projectToDisks({.5, 0}, {{0, 0}, {1, 0}}, 1, projected) && dist(projected, {.5, 0}) < EPS,
            "projection preserves feasible points");
    require(projectToDisks({1, 2}, {{0, 0}, {2, 0}}, 1, projected) && dist(projected, {1, 0}) < EPS,
            "projection handles tangent disks");
    auto checkExactRoute = [](Point start, const std::map<int, Point>& destinations) {
        vector<int> expected;
        for (const auto& item : destinations) expected.push_back(item.first);
        vector<int> order = exactRoute(start, destinations), sorted = order;
        std::sort(sorted.begin(), sorted.end());
        require(sorted == expected, "exact route visits each channel once");
        auto length = [&](const vector<int>& routeOrder) {
            Point position = start;
            double result = 0;
            for (int channel : routeOrder) {
                Point next = destinations.at(channel);
                result += dist(position, next);
                position = next;
            }
            return result;
        };
        double optimum = std::numeric_limits<double>::infinity();
        do { optimum = std::min(optimum, length(expected)); }
        while (std::next_permutation(expected.begin(), expected.end()));
        require(std::abs(length(order) - optimum) <= 1e-8 * std::max(1.0, optimum),
                "exact open route versus exhaustive permutations");
    };
    checkExactRoute({0, 0}, {});
    checkExactRoute({-3, 2}, {{7, {1, 1}}});
    checkExactRoute({0, 0}, {{2, {0, 0}}, {5, {10, 0}}, {19, {10, 0}}});
    checkExactRoute({0, 0}, {{1, {1, 0}}, {3, {10, 0}}, {20, {0, 2}}});
    std::mt19937 routeRandom(20260913);
    std::uniform_real_distribution<double> routeCoordinate(-100, 100);
    for (int trial = 0; trial < 12; ++trial) {
        std::map<int, Point> destinations;
        for (int i = 0; i < 2 + trial % 6; ++i)
            destinations[2 * i + 1] = {routeCoordinate(routeRandom), routeCoordinate(routeRandom)};
        checkExactRoute({routeCoordinate(routeRandom), routeCoordinate(routeRandom)}, destinations);
    }
    vector<Point> stations = searchPoints();
    const double ringRadius = 1125.0;
    const double worstBoundaryDistance = std::sqrt(1800.0 * 1800.0 + ringRadius * ringRadius
        - 2 * 1800.0 * ringRadius * std::cos(PI / 6));
    require(worstBoundaryDistance < 1000.0, "analytic ring coverage margin");
    for (int r = 0; r <= 1800; r += 100) {
        for (int angle = 0; angle < 3600; ++angle) {
            Point source = unit(angle * .1) * r;
            double nearest = 1e100;
            for (Point p : stations) nearest = std::min(nearest, dist(p, source));
            require(nearest <= 1000, "seven-point coverage");
        }
    }
    Circle triangle = mec({{0, 0}, {2, 0}, {1, std::sqrt(3.0)}});
    require(std::abs(triangle.r - 2 / std::sqrt(3.0)) < 1e-6, "triangle MEC");
    Circle line = mec({{0, 0}, {1, 0}, {3, 0}, {2, 0}});
    require(std::abs(line.r - 1.5) < 1e-6, "collinear MEC");
    std::mt19937 random(727);
    std::uniform_real_distribution<double> coord(-10, 10);
    for (int trial = 0; trial < 80; ++trial) {
        Poly points;
        for (int k = 0; k < 8; ++k) points.push_back({coord(random), coord(random)});
        double best = 1e100;
        auto accept = [&](Circle c) {
            for (Point p : points) if (!c.contains(p)) return;
            best = std::min(best, c.r);
        };
        for (size_t i = 0; i < points.size(); ++i) for (size_t j = i + 1; j < points.size(); ++j) {
            accept(diameter(points[i], points[j]));
            for (size_t k = j + 1; k < points.size(); ++k) {
                Circle c;
                if (circum(points[i], points[j], points[k], c)) accept(c);
            }
        }
        require(std::abs(mec(points).r - best) < 1e-6, "MEC versus exhaustive support circles");
    }
    Mock mock;
    mock.sources[3] = {{300, 0}, 1000};
    Session session(mock, "TEST", "");
    session.enter();
    session.measure({300, 400}, 1);
    session.measure({300, 400}, 2);
    session.clear({300, 0}, 3);
    session.measure({300, 0}, 2);
    session.exit();
    require(session.current == 2 && session.switches == 1, "/clear must not switch channels");
    require(std::abs(session.vt - 201) < 1e-6, "movement/switch/clear time accounting");
    for (int mode = 0; mode < 4; ++mode) for (double radius : {20.1, 35.0, 39.7}) {
        for (double angle : {0.0, 89.99, 180.0, 359.99}) {
            Mock centerMock;
            centerMock.mode = mode;
            Point center{100, -100};
            centerMock.sources[4] = {center + unit(angle) * radius, 1000};
            Session centerSession(centerMock, "TEST", "");
            Planner centerPlanner(centerSession, true);
            centerSession.enter();
            Target& target = centerPlanner.targets[4];
            target.channel = 4;
            target.region = outerDisk(center, radius);
            target.c = mec(target.region);
            centerPlanner.observe(target.c.c, 4);
            require(target.status == "located", "one center reading certifies a region below 39.79 m");
            Point clear = centerPlanner.clearPoint(target, centerSession.pos);
            require(centerPlanner.farthest(target, clear) <= 19.9 + 1e-6, "nearer clearing point contains the region");
            centerPlanner.clearTarget(target, clear);
            centerSession.exit();
            require(centerMock.cleared.count(4) == 1, "center reading followed by certified clearing");
        }
    }
    for (int mode = 0; mode < 4; ++mode) for (double range : {6.0, 30.0, 751.0, 1500.0}) {
        Mock homingMock;
        homingMock.mode = mode;
        homingMock.sources[4] = {unit(359.99) * range, 1500};
        Session homingSession(homingMock, "TEST", "");
        Planner homingPlanner(homingSession, true);
        homingSession.enter();
        Target* target = homingPlanner.observe({0, 0}, 4);
        require(target != nullptr, "initial homing observation");
        homingPlanner.homing(*target);
        if (target->status != "cleared") homingPlanner.clearTarget(*target, target->c.c);
        homingSession.exit();
        require(homingMock.cleared.count(4) == 1, "homing reaches clear radius");
    }
}

#endif
