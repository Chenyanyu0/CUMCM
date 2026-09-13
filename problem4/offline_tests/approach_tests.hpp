#pragma once

#ifdef APPROACH_TEST_STANDALONE
void approachTests();
#include "../planner.hpp"
#include "tests.hpp"
#endif

void approachTests() {
    const Point first{-500, 0}, second{500, 0}, actual{0, 800};
    Mock mock;
    mock.mode = 3;
    mock.sources[4] = {actual, 1000, true, 270};
    Session session(mock, "APPROACH-TEST", "");
    Planner planner(session, true);
    mock.beforeAction = [&]() { checkTargetContainment(planner, mock); };
    session.enter();
    Target* target = planner.observe(first, 4);
    require(target && target->obs.size() == 1, "approach fixture has its first received bearing");
    require(planner.approachCandidates(*target).empty() && planner.choose(*target).kind != "approach",
            "one received bearing cannot enable a two-anchor approach");
    require(planner.observe(second, 4) && target->obs.size() == 2,
            "second received bearing completes the approach certificate");
    require(target->c.r > 20, "symmetric two-bearing region still requires localization");
    const Target initial = *target;
    const auto candidates = planner.approachCandidates(*target);
    require(!candidates.empty(), "two bearings produce certified close-approach alternatives");
    for (const auto& candidate : candidates) {
        require(approach_geometry::certified(candidate.point, first, second, target->region, 0),
                "generated approach point has an independent triangle certificate");
        require(mock.receives(mock.sources.at(4), candidate.point),
                "directional source receives every generated approach point");
    }
    Planner::Plan plan = planner.choose(*target);
    require(plan.kind == "approach", "two-anchor localization prioritizes a productive close approach");
    require(std::abs(plan.entry.x) > 1,
            "long symmetric source region selects parallax instead of its ambiguous central axis");
    require(dist(plan.entry, actual) < dist(second, actual) * .5,
            "selected probe approaches the source region substantially");
    double radiusBefore = target->c.r;
    int measuresBefore = session.measures;
    int gBefore = planner.gMeasurements, fBefore = planner.fMeasurements;
    planner.approachStep(*target, plan);
    require(session.measures == measuresBefore + 1 && planner.approachMeasures == 1,
            "one approach action performs exactly one RF measurement");
    require(planner.gMeasurements == gBefore && planner.fMeasurements == fBefore,
            "successful approach does not expand into old F/G probes");
    require(target->status != "detected" || target->c.r < .7 * radiusBefore,
            "nearby lateral bearing contracts the actual region substantially");
    checkTargetContainment(planner, mock);
    for (int step = 0; step < 20 && target->status != "cleared"; ++step)
        planner.localStep(*target);
    require(target->status == "cleared" && session.failed == 0,
            "approach localization reaches a certified successful clear");
    session.exit();

    Mock policyMock;
    Session policySession(policyMock, "APPROACH-POLICY", "");
    Planner policyPlanner(policySession, true);
    for (int limit : {0, 1, 2}) {
        Target limited = initial;
        if (limit == 0) limited.approachAttempts = 6;
        if (limit == 1) limited.approachStalls = 2;
        if (limit == 2) limited.approachBlocked = true;
        require(policyPlanner.approachCandidates(limited).empty() &&
                    policyPlanner.choose(limited).kind == "G",
                "attempt, stagnation, and reception-failure limits preserve paired G fallback");
    }
    policyPlanner.closeApproach = false;
    require(policyPlanner.choose(initial).kind != "approach", "disabled mode restores the older probe policy");

    Mock stallMock;
    stallMock.mode = 3;
    stallMock.sources[4] = {actual, 1000, true, 270};
    Session stallSession(stallMock, "APPROACH-STALL", "");
    Planner stallPlanner(stallSession, true);
    stallSession.enter();
    stallPlanner.observe(first, 4);
    Target* stalled = stallPlanner.observe(second, 4);
    for (Point axial : {Point{0, 0}, Point{0, 10}}) {
        Planner::Plan axialPlan;
        axialPlan.kind = "approach";
        axialPlan.entry = axial;
        axialPlan.anchorFirst = 0;
        axialPlan.anchorSecond = 1;
        stallPlanner.approachStep(*stalled, axialPlan);
    }
    require(stalled->approachStalls == 2 && stalled->approachAttempts == 2 &&
                stallPlanner.choose(*stalled).kind == "G",
            "two actual axial measurements with negligible diameter improvement trigger fallback");
    checkTargetContainment(stallPlanner, stallMock);
    stallSession.exit();

    // Inject a contradictory response to exercise the conservative failure path.
    Mock faultMock;
    faultMock.mode = 3;
    faultMock.sources[4] = {actual, 1000, true, 270};
    Session faultSession(faultMock, "APPROACH-FAULT", "");
    Planner faultPlanner(faultSession, true);
    faultSession.enter();
    faultPlanner.observe(first, 4);
    Target* faultTarget = faultPlanner.observe(second, 4);
    Planner::Plan faultPlan = faultPlanner.choose(*faultTarget);
    require(faultPlan.kind == "approach", "fault fixture prepares a certified approach");
    Poly faultRegion = faultTarget->region;
    vector<double> faultBounds = faultTarget->upperBounds;
    faultMock.sources[4].direction = 90;
    faultPlanner.approachStep(*faultTarget, faultPlan);
    require(faultPlanner.approachMisses == 1 && faultTarget->approachBlocked &&
                faultPlanner.choose(*faultTarget).kind == "G",
            "unexpected no_signal disables further approaches and preserves fallback");
    require(faultRegion.size() == faultTarget->region.size() && faultBounds == faultTarget->upperBounds,
            "unexpected approach miss preserves the certified bounds");
    for (size_t i = 0; i < faultRegion.size(); ++i)
        require(dist(faultRegion[i], faultTarget->region[i]) < 1e-10,
                "unexpected approach miss does not clip the source region");
    faultSession.exit();

    int visibilityChecks = 0, boundaryChecks = 0;
    for (double radius : {1000.0, 1500.0}) {
        Mock rangeMock;
        rangeMock.mode = 3;
        Point source = radius == 1000 ? Point{0, 800} : Point{0, 1400};
        Point a = radius == 1000 ? Point{-600, 0} : Point{-200, 0};
        Point b = radius == 1000 ? Point{600, 0} : Point{200, 0};
        rangeMock.sources[4] = {source, radius, false, 0};
        Session rangeSession(rangeMock, "APPROACH-RANGE", "");
        Planner rangePlanner(rangeSession, true);
        rangeSession.enter();
        require(rangePlanner.observe(a, 4) && rangePlanner.observe(b, 4),
                "range-boundary anchors both receive the source");
        const Target& ranged = rangePlanner.targets.at(4);
        const auto rangeCandidates = rangePlanner.approachCandidates(ranged);
        require(!rangeCandidates.empty(), "range fixture generates approach candidates");
        vector<Point> sources = ranged.region;
        sources.push_back(source);
        sources.push_back(ranged.c.c);
        for (Point possible : sources) {
            vector<double> headings;
            for (int angle = 0; angle < 360; angle += 15) headings.push_back(angle);
            for (Point anchor : {a, b}) {
                headings.push_back(bearing(possible, anchor) - 90);
                headings.push_back(bearing(possible, anchor) + 90);
            }
            for (double heading : headings) {
                Mock::Source variant{possible, radius, true, heading};
                if (!rangeMock.receives(variant, a) || !rangeMock.receives(variant, b)) continue;
                for (const auto& candidate : rangeCandidates) {
                    require(rangeMock.receives(variant, candidate.point),
                            "sampled feasible source position and heading receive every candidate");
                    ++visibilityChecks;
                    if (std::abs(dot(a - possible, unit(heading))) < 1e-6 ||
                        std::abs(dot(b - possible, unit(heading))) < 1e-6) ++boundaryChecks;
                }
            }
        }
        if (radius == 1500) {
            Point probe{0, 1};
            require(approach_geometry::certified(probe, a, b, ranged.region, 0) &&
                        dist(probe, source) > 1000 && rangeMock.receives(rangeMock.sources.at(4), probe),
                    "successful anchors certify reception beyond the 1000 m minimum radius");
        }
        rangeSession.exit();
    }
    require(visibilityChecks > 100 && boundaryChecks > 0,
            "visibility sampling covers interior and closed emission-boundary orientations");

    const Point a{0, 0}, b{200, 0};
    Poly asymmetric{{272.07925546002014, 771.1093157888413},
                    {284.7550501660687, 906.7159243821087},
                    {339.08387580134206, 1079.7095598865615},
                    {315.00128064156956, 892.7561257011082}};
    const Point unsafe{272.642797880134, 777.1381351380237};
    Mock boundaryMock;
    Mock::Source downward{asymmetric[0], 1000, true, 270};
    require(testContains(asymmetric, unsafe) && boundaryMock.receives(downward, a) &&
                boundaryMock.receives(downward, b) && !boundaryMock.receives(downward, unsafe),
            "midpoint-to-center boundary counterexample has two visible anchors and invisible destination");
    require(!approach_geometry::certified(unsafe, a, b, asymmetric),
            "unsafe candidate-region boundary is never certified as a close approach");
    Poly safe = approach_geometry::safeRegion(a, b, asymmetric);
    require(!safe.empty(), "asymmetric region still has a conservative approach corridor");
    Point projected = approach_geometry::project(unsafe, safe);
    require(approach_geometry::certified(projected, a, b, asymmetric) &&
                boundaryMock.receives(downward, projected),
            "projection stops inside the shared reception triangle before the unsafe boundary");
    std::reverse(asymmetric.begin(), asymmetric.end());
    require(!approach_geometry::safeRegion(b, a, asymmetric).empty(),
            "source winding and anchor order do not affect corridor existence");

    for (int kind = 0; kind < 4; ++kind) {
        Target degenerate = initial;
        if (kind == 0) degenerate.obs[1].first = degenerate.obs[0].first;
        if (kind == 1) degenerate.region = {{-100, -50}, {100, -50}, {100, 50}, {-100, 50}};
        if (kind == 2) degenerate.region = {{0, 700}, {0, 800}, {0, 900}};
        if (kind == 3) degenerate.region = {{-50, 0}, {50, 0}, {50, 100}, {-50, 100}};
        policyPlanner.refresh(degenerate);
        policyPlanner.closeApproach = true;
        require(policyPlanner.approachCandidates(degenerate).empty() &&
                    policyPlanner.choose(degenerate).kind == "G",
                "coincident anchors, crossing/touching baselines, and zero-area polygons use G fallback");
    }
}

#ifdef APPROACH_TEST_STANDALONE
int main() {
    try {
        approachTests();
        std::cout << "approach tests OK\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
#endif
