#ifndef PROBLEM3_TESTS_NEIGHBORHOOD_ROUTE_TESTS_HPP
#define PROBLEM3_TESTS_NEIGHBORHOOD_ROUTE_TESTS_HPP

// Offline tests only. Include neighborhood_route.hpp before this test header.

inline void neighborhoodRouteTests() {
    auto require = [](bool condition, const char* message) {
        if (!condition) throw std::runtime_error(message);
    };
    auto option = [](Point entry, Point exit, int channel, double service) {
        RouteOption result;
        result.entry = entry; result.exit = exit;
        result.channel = channel; result.service = service;
        return result;
    };
    const Point start{0, 0};
    RouteGroup region;
    region.id = 51;
    region.options = {option({10, 10}, {10, 10}, 1, 5),
                      option({10, -10}, {10, -10}, 1, 5)};
    Point upper{20, 20}, lower{20, -20};
    RouteDecision a = solveNeighborhoodRoute(start, 1, {region}, false, &upper);
    RouteDecision b = solveNeighborhoodRoute(start, 1, {region}, false, &lower);
    require(a.group == 0 && a.option == 0 && b.option == 1,
            "Changing the onward anchor must change the region entry");
    RouteDecision direct = solveNeighborhoodRoute(start, 1, {region}, true, &upper);
    require(direct.group == -1 && direct.visits == 0 &&
            std::abs(direct.cost - dist(start, upper) / 5) < 1e-9,
            "Optional neighborhood route must retain the direct alternative");

    RouteGroup first, second;
    first.options = {option({50, 0}, {50, 0}, 1, -12)};
    second.options = {option({55, 0}, {55, 0}, 1, -12)};
    RouteDecision pair = solveNeighborhoodRoute(start, 1, {first, second}, true, &start);
    require(pair.visits == 2 && std::abs(pair.cost + 2) < 1e-9,
            "Two jointly worthwhile regions must survive unfavorable individual detours");

    RouteGroup clear, measure;
    clear.options = {option(start, start, 0, -2)};
    measure.options = {option(start, start, 7, 0)};
    RouteDecision same = solveNeighborhoodRoute(start, 7, {clear, measure}, false, &start, 7);
    require(std::abs(same.cost + 2) < 1e-9,
            "A clear action must preserve the active channel");
    RouteDecision switched = solveNeighborhoodRoute(start, 3, {clear, measure}, false, &start, 7);
    require(std::abs(switched.cost + 1) < 1e-9,
            "The route must pay exactly one change from channel three to seven");
    RouteDecision tailSwitch = solveNeighborhoodRoute(start, 7, {clear}, false, &start, 3);
    require(std::abs(tailSwitch.cost + 1) < 1e-9,
            "The onward channel must be charged after a clear action");
    RouteDecision impossible = solveNeighborhoodRoute(start, 1, {RouteGroup{}}, false);
    require(impossible.group == -1 && !std::isfinite(impossible.cost),
            "A required group without options must be infeasible");
    RouteGroup station, reward;
    station.options = {option({10, 0}, {10, 0}, 1, 25)};
    reward.options = {option({5, 0}, {5, 0}, 1, -2)};
    RouteDecision mandatory = solveNeighborhoodRoute(start, 1, {station, reward}, true,
                                                      nullptr, 0, 1);
    require(mandatory.visits == 2 && mandatory.group == 1 &&
            mandatory.choices.back().first == 0 && std::abs(mandatory.cost - 25) < 1e-9 &&
            std::abs(mandatory.distance - 10) < 1e-9,
            "Optional rewards cannot replace a required positive-cost station");
    RouteDecision mandatoryOnly = solveNeighborhoodRoute(start, 1, {station}, true,
                                                          nullptr, 0, 1);
    require(mandatoryOnly.visits == 1 && std::abs(mandatoryOnly.cost - 27) < 1e-9,
            "A mandatory station must not lose to an infeasible empty route");
    RouteDecision missingStation = solveNeighborhoodRoute(start, 1, {RouteGroup{}, reward}, true,
                                                          nullptr, 0, 1);
    require(!std::isfinite(missingStation.cost) && missingStation.choices.empty(),
            "An empty mandatory group cannot be skipped for an optional reward");
    RouteGroup scan;
    scan.options = {option(start, start, 1, 6)};
    scan.options.front().endChannel = 7;
    RouteDecision scanThenMeasure = solveNeighborhoodRoute(start, 1, {scan, measure}, false,
                                                           &start, 7);
    require(std::abs(scanThenMeasure.cost - 6) < 1e-9 && scanThenMeasure.group == 0,
            "A multi-channel scan must expose its ending channel to the next action");
    RouteDecision budgetScan = solveBudgetedNeighborhoodRoute(start, 1, {scan}, start, 7, 0);
    require(budgetScan.visits == 0 && std::abs(budgetScan.cost - 1) < 1e-9,
            "A positive-cost optional scan must not defeat the direct route");
    scan.options.front().service = -6;
    budgetScan = solveBudgetedNeighborhoodRoute(start, 1, {scan, measure}, start, 7, 0);
    require(budgetScan.visits == 1 && std::abs(budgetScan.cost + 6) < 1e-9 &&
            budgetScan.choices.size() == 1 && budgetScan.distance == 0,
            "Budgeted scans must account for their ending channel");
    std::vector<RouteGroup> tenStations(10);
    for (int i = 0; i < 10; ++i)
        tenStations[i].options = {option({double(i + 1), 0}, {double(i + 1), 0}, 0, 1)};
    RouteDecision ten = solveNeighborhoodRoute(start, 1, tenStations, false);
    require(ten.visits == 10 && ten.choices.size() == 10 && ten.group == 0 &&
            std::abs(ten.cost - 12) < 1e-9 && std::abs(ten.distance - 10) < 1e-9,
            "The rolling window must support ten mandatory groups");
    scan.options.front().endChannel = 21;
    bool invalidEnding = false;
    try { solveNeighborhoodRoute(start, 1, {scan}, false); }
    catch (const std::invalid_argument&) { invalidEnding = true; }
    require(invalidEnding, "Invalid scan ending channels must be rejected");

    Point budgetAnchor{100, 0};
    RouteGroup above, below;
    above.options = {option({50, 30}, {50, 30}, 1, -30)};
    below.options = {option({50, -30}, {50, -30}, 1, -30)};
    RouteDecision unlimited = solveNeighborhoodRoute(start, 1, {above, below}, true, &budgetAnchor);
    RouteDecision limited = solveBudgetedNeighborhoodRoute(start, 1, {above, below}, budgetAnchor, 0, 20);
    require(unlimited.visits == 2 && limited.visits == 1 &&
            std::abs(limited.cost - (2 * std::hypot(50.0, 30.0) / 5 - 30)) < 1e-9,
            "Individually feasible detours must not form an overbudget combined route");
    RouteDecision zeroBudget = solveBudgetedNeighborhoodRoute(start, 1, {above, below}, budgetAnchor, 0, 0);
    require(zeroBudget.visits == 0 && zeroBudget.group == -1 && std::abs(zeroBudget.cost - 20) < 1e-9,
            "A zero detour budget must retain a feasible direct route");
    RouteDecision budgetPair = solveBudgetedNeighborhoodRoute(start, 1, {first, second}, start, 0, 110);
    require(budgetPair.visits == 2 && std::abs(budgetPair.cost + 2) < 1e-9,
            "Budgeted route must retain profitable combinations of individually unprofitable stops");
    RouteDecision budgetChannel = solveBudgetedNeighborhoodRoute(start, 7, {clear, measure}, start, 3, 0);
    require(std::abs(budgetChannel.cost + 1) < 1e-9,
            "Budgeted clear actions must preserve the channel before the onward switch");

    // Exhaust all orders and options independently on several small problems.
    // Keeping the best cost per first action also checks returned indices.
    for (int fixture = 0; fixture < 8; ++fixture) {
        std::vector<RouteGroup> groups(3);
        for (int i = 0; i < 3; ++i) {
            groups[i].id = 100 - i;
            for (int j = 0; j < 2; ++j) {
                Point entry{double((i * 17 + j * 11 + fixture * 3) % 29 - 14),
                            double((i * 13 - j * 7 + fixture * 5 + 30) % 31 - 15)};
                Point exit{entry.x + i - j, entry.y + j * 3};
                groups[i].options.push_back(option(entry, exit, (i + j + fixture) % 4,
                                                   double((fixture + 2 * i + j) % 9 - 6)));
                if ((i + j + fixture) % 3 == 0)
                    groups[i].options.back().endChannel = (fixture + i + j) % 5 + 1;
            }
        }
        Point end{double(fixture * 7 - 20), double(10 - fixture * 3)};
        for (bool optional : {false, true}) {
            for (bool anchored : {false, true}) {
              for (int required : {-1, 0, 1, 3, 7}) {
                const int current = fixture % 3 + 1;
                const int onward = fixture % 4;
                const Point* anchor = anchored ? &end : nullptr;
                RouteDecision dp = solveNeighborhoodRoute(start, current, groups, optional, anchor, onward,
                                                           required);
                const int mandatoryBits = optional ? std::max(0, required) : 7;
                double best = std::numeric_limits<double>::infinity();
                double returnedFirstBest = std::numeric_limits<double>::infinity();
                std::function<void(int, Point, int, double, int, int)> enumerate;
                enumerate = [&](int mask, Point from, int channel, double cost, int firstGroup, int firstOption) {
                    if ((mask & mandatoryBits) == mandatoryBits) {
                        const double total = cost + (anchor ? dist(from, *anchor) / 5 +
                            (onward != 0 && channel != onward ? 1 : 0) : 0.0);
                        best = std::min(best, total);
                        if (firstGroup == dp.group && firstOption == dp.option)
                            returnedFirstBest = std::min(returnedFirstBest, total);
                    }
                    for (int i = 0; i < 3; ++i) {
                        if (mask & (1 << i)) continue;
                        for (int j = 0; j < 2; ++j) {
                            const RouteOption& candidate = groups[i].options[j];
                            const int actionChannel = candidate.channel == 0 ? channel : candidate.channel;
                            const int nextChannel = candidate.endChannel == 0 ? actionChannel : candidate.endChannel;
                            enumerate(mask | (1 << i), candidate.exit, nextChannel,
                                cost + dist(from, candidate.entry) / 5 + candidate.service +
                                    (channel != actionChannel),
                                mask == 0 ? i : firstGroup, mask == 0 ? j : firstOption);
                        }
                    }
                };
                enumerate(0, start, current, 0, -1, -1);
                require(std::abs(dp.cost - best) < 1e-8 &&
                        std::abs(dp.cost - returnedFirstBest) < 1e-8,
                        "Neighborhood dynamic program must match exhaustive routes");
                int reconstructedMask = 0, reconstructedChannel = current;
                double reconstructedDistance = 0, reconstructedService = 0;
                Point previous = start;
                for (const auto& choice : dp.choices) {
                    require(choice.first >= 0 && choice.first < 3 && choice.second >= 0 && choice.second < 2 &&
                            !(reconstructedMask & (1 << choice.first)),
                            "Reconstructed route must contain distinct valid groups");
                    reconstructedMask |= 1 << choice.first;
                    const RouteOption& candidate = groups[choice.first].options[choice.second];
                    reconstructedDistance += dist(previous, candidate.entry);
                    reconstructedService += candidate.service +
                        (candidate.channel != 0 && candidate.channel != reconstructedChannel);
                    if (candidate.channel != 0) reconstructedChannel = candidate.channel;
                    if (candidate.endChannel != 0) reconstructedChannel = candidate.endChannel;
                    previous = candidate.exit;
                }
                if (anchor) {
                    reconstructedDistance += dist(previous, *anchor);
                    reconstructedService += onward != 0 && onward != reconstructedChannel;
                }
                require((reconstructedMask & mandatoryBits) == mandatoryBits &&
                        int(dp.choices.size()) == dp.visits &&
                        std::abs(dp.distance - reconstructedDistance) < 1e-8 &&
                        std::abs(dp.cost - reconstructedDistance / 5 - reconstructedService) < 1e-8,
                        "Reconstructed route must satisfy required tasks and reproduce cost and distance");
              }
            }
        }
        for (RouteGroup& group : groups)
            for (RouteOption& candidate : group.options) candidate.exit = candidate.entry;
        for (double budget : {0.0, 5.0, 20.0, 100.0}) {
            const int current = fixture % 3 + 1;
            const int onward = fixture % 4;
            RouteDecision bounded = solveBudgetedNeighborhoodRoute(start, current, groups, end, onward, budget);
            double best = std::numeric_limits<double>::infinity();
            double returnedFirstBest = std::numeric_limits<double>::infinity();
            std::function<void(int, Point, int, double, double, int, int)> enumerate;
            enumerate = [&](int mask, Point from, int channel, double travel, double service,
                            int firstGroup, int firstOption) {
                const double distance = travel + dist(from, end);
                if (distance <= dist(start, end) + budget + 1e-7) {
                    const double total = distance / 5 + service + (onward != 0 && channel != onward);
                    best = std::min(best, total);
                    if (firstGroup == bounded.group && firstOption == bounded.option)
                        returnedFirstBest = std::min(returnedFirstBest, total);
                }
                // Deliberately enumerate even overbudget prefixes as an
                // independent check of the production search's pruning.
                for (int i = 0; i < 3; ++i) {
                    if (mask & (1 << i)) continue;
                    for (int j = 0; j < 2; ++j) {
                        const RouteOption& candidate = groups[i].options[j];
                        const int actionChannel = candidate.channel == 0 ? channel : candidate.channel;
                        const int nextChannel = candidate.endChannel == 0 ? actionChannel : candidate.endChannel;
                        enumerate(mask | (1 << i), candidate.exit, nextChannel,
                            travel + dist(from, candidate.entry),
                            service + candidate.service + (channel != actionChannel),
                            mask == 0 ? i : firstGroup, mask == 0 ? j : firstOption);
                    }
                }
            };
            enumerate(0, start, current, 0, 0, -1, -1);
            require(std::abs(bounded.cost - best) < 1e-8 &&
                    std::abs(bounded.cost - returnedFirstBest) < 1e-8,
                    "Budgeted neighborhood route must match independent exhaustive routes");
        }
    }
}

#endif
