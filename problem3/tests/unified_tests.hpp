// Offline test code; included by tests/geometry_tests.hpp.
#ifndef PROBLEM3_UNIFIED_TESTS_HPP
#define PROBLEM3_UNIFIED_TESTS_HPP

// Included after Planner, Mock and require in main.cpp.
inline void unifiedPlannerTests() {
    struct RecordingMock : Mock {
        std::vector<std::pair<std::string, Json>> calls;
        std::string post(const std::string& path, const std::string& body, double deadline) override {
            std::string response = Mock::post(path, body, deadline);
            calls.emplace_back(path, Json::parse(body));
            return response;
        }
    };
    auto requestPoint = [](const Json& request) {
        return Point{request.at("position").at("x").get<double>(),
                     request.at("position").at("y").get<double>()};
    };
    auto containsSource = [](const Target& target, Point source) {
        if (target.region.empty()) return target.status == "cleared";
        for (size_t i = 0; i < target.region.size(); ++i) {
            Point a = target.region[i];
            Point edge = target.region[(i + 1) % target.region.size()] - a;
            if (cross(edge, source - a) < -1e-5) return false;
        }
        return target.c.contains(source);
    };
    auto timeMatches = [](const Session& session) {
        require(std::abs(session.summary().at("time_accounting_error_s").get<double>()) < .001,
                "unified integration actions must match the virtual clock");
    };

    {
        RecordingMock mock;
        mock.mode = 3;
        mock.sources[7] = {{1000, 0}, 1500};
        Session session(mock, "TEST", "");
        Planner planner(session, true);
        session.enter();
        Target* target = planner.observe({0, 0}, 7);
        require(target && target->obs.size() == 1 && !target->secondRegion.empty() &&
                planner.areaCreated == 1, "first real bearing must create an S2 region task");
        auto area = planner.areaCandidates(*target, {{0, 0}, {1125, 0}}, 8);
        require(area.size() > 1, "an S2 region must retain alternative executable positions");
        for (Point point : area)
            require(planner.guaranteedReception(*target, point), "S2 alternatives must guarantee reception");

        // A short baseline intentionally leaves a region requiring a third reading.
        require(planner.observe({0, 100}, 7) != nullptr, "second real bearing must receive the source");
        require(target->obs.size() == 2 && target->secondRegion.empty() &&
                planner.areaRetired == 1 && target->status == "detected" && target->c.r > 19.9,
                "second bearing must retire S2 while preserving a refinement task");
        require(containsSource(*target, mock.sources.at(7).position),
                "two observed wedges must retain the real source");
        auto candidates = planner.taskCandidates(*target, {session.pos, {1125, 0}, {0, -1125}});
        require(candidates.size() > 1, "two-bearing refinement must retain multiple execution points");
        auto best = std::min_element(candidates.begin(), candidates.end(),
            [](const Planner::CandidatePrediction& a, const Planner::CandidatePrediction& b) {
                return a.prediction.remainingTime < b.prediction.remainingTime;
            });
        Point chosen = best->point;
        double oldRadius = target->c.r;
        size_t firstCall = mock.calls.size();
        int oldMeasures = session.measures;
        planner.executeUnifiedTarget(*target, chosen);
        require(mock.calls.size() > firstCall && mock.calls[firstCall].first == "/measure" &&
                mock.calls[firstCall].second.at("channel").get<int>() == 7 &&
                dist(requestPoint(mock.calls[firstCall].second), chosen) < 1e-8,
                "refinement must execute its selected point without choosing a replacement");
        require(session.measures > oldMeasures &&
                (target->status == "cleared" || target->c.r < oldRadius),
                "a real third reading must reduce uncertainty or clear the source");
        require(containsSource(*target, mock.sources.at(7).position),
                "third-reading refinement must retain the real source");
        std::set<int> unknown;
        std::vector<bool> visited(planner.search.size(), true);
        planner.scanned = static_cast<int>(planner.search.size());
        for (int step = 0; step < 10 && target->status != "cleared"; ++step)
            require(planner.unifiedDispatch(unknown, visited), "remaining refinement must remain schedulable");
        require(target->status == "cleared" && mock.cleared.count(7) && session.failed == 0 &&
                planner.unifiedGuaranteedMisses == 0, "refinement must finish with a certified real clear");
        timeMatches(session);
        session.exit();
    }

    {
        RecordingMock mock;
        Session session(mock, "TEST", "");
        Planner planner(session, true);
        session.enter();
        planner.observe({0, 50}, 7);
        std::set<int> unknown{2, 7, 11};
        const std::set<int> originalUnknown = unknown;
        std::vector<bool> visited(planner.search.size(), false);
        Point before = session.pos;
        double beforeTime = session.vt;
        int beforeMeasures = session.measures, beforeSwitches = session.switches;
        size_t firstCall = mock.calls.size();
        require(planner.unifiedDispatch(unknown, visited), "unknown channels must produce a search action");
        const RouteDecision& decision = planner.lastDecision;
        const RouteGroup& selected = planner.lastWindow.at(decision.group);
        const RouteOption& option = selected.options.at(decision.option);
        require(selected.id < 0, "a window without sources must choose a search station");
        std::vector<int> expected = planner.scanChannels(originalUnknown, option.channel);
        require(mock.calls.size() == firstCall + expected.size(),
                "one search task must execute exactly its unknown-channel scan");
        for (size_t i = 0; i < expected.size(); ++i)
            require(mock.calls[firstCall + i].first == "/measure" &&
                    mock.calls[firstCall + i].second.at("channel").get<int>() == expected[i] &&
                    dist(requestPoint(mock.calls[firstCall + i].second), option.entry) < 1e-8,
                    "search action channel order and station must match its chosen option");
        require(session.current == option.endChannel && session.current == expected.back(),
                "a real station scan must finish on its planned ending channel");
        int expectedSwitches = static_cast<int>(expected.size()) - 1 + (option.channel != 7);
        require(session.measures - beforeMeasures == static_cast<int>(expected.size()) &&
                session.switches - beforeSwitches == expectedSwitches &&
                std::abs(session.vt - beforeTime - dist(before, option.entry) / 5 -
                         5 * expected.size() - expectedSwitches) < 1e-5,
                "search service must include every measurement and actual channel switch");
        require(unknown == originalUnknown && planner.scanned == 1 &&
                std::count(visited.begin(), visited.end(), true) == 1,
                "one negative station scan must not certify an unknown channel absent");
        timeMatches(session);
        session.exit();
    }

    {
        RecordingMock mock;
        mock.mode = 3;
        mock.sources[9] = {{900, 0}, 1500};
        Session session(mock, "TEST", "");
        Planner planner(session, true);
        session.enter();
        planner.observe({850, 0}, 9);
        Target* target = planner.observe({900, 50}, 9);
        require(target && target->status == "located", "guard fixture must have a pending clear task");
        planner.actionsSinceSearch = 8;
        std::set<int> unknown{20};
        std::vector<bool> visited(planner.search.size(), false);
        require(planner.unifiedDispatch(unknown, visited), "search progress guard must produce an action");
        const auto& groups = planner.lastWindow;
        require(!groups.empty() && std::all_of(groups.begin(), groups.end(),
                    [](const RouteGroup& group) { return group.id < 0; }) &&
                groups.at(planner.lastDecision.group).id < 0 && planner.scanned == 1 &&
                planner.actionsSinceSearch == 0,
                "after eight target actions, coverage must advance before another target action");
        require(target->status == "located" && session.failed == 0,
                "forced search must preserve the outstanding certified clear task");
        timeMatches(session);
        session.exit();
    }

    {
        RecordingMock mock;
        mock.mode = 3;
        mock.sources[7] = {{900, 300}, 1500};
        mock.sources[8] = {{1000, 0}, 1500};
        mock.sources[9] = {{450, -450}, 1500};
        Session session(mock, "TEST", "");
        Planner planner(session, true);
        session.enter();
        planner.observe({0, 0}, 7);
        planner.observe({0, 0}, 8);
        planner.observe({0, 100}, 8);
        planner.observe({400, -450}, 9);
        planner.observe({450, -400}, 9);
        const std::map<int, Target> before = planner.targets;
        require(before.at(7).obs.size() == 1 && !before.at(7).secondRegion.empty() &&
                before.at(8).obs.size() == 2 && before.at(8).secondRegion.empty() &&
                before.at(8).status == "detected" && before.at(9).status == "located",
                "mixed fixture must contain S2, true-region refinement and clear tasks");
        std::set<int> unknown{19, 20};
        std::vector<bool> visited(planner.search.size(), false);
        size_t firstCall = mock.calls.size();
        require(planner.unifiedDispatch(unknown, visited), "mixed task window must execute an action");
        const auto& groups = planner.lastWindow;
        const RouteDecision& decision = planner.lastDecision;
        std::set<int> taskIds;
        for (const RouteGroup& group : groups) {
            require(taskIds.insert(group.id).second, "a unified window must contain one group per task");
            if (group.id < 0) continue;
            require(group.options.size() > 1,
                    "mixed-region planning must preserve execution alternatives until route selection");
            const Target& previous = before.at(group.id);
            bool lowerAreaBranch = false, upperAreaBranch = false;
            for (const RouteOption& option : group.options) {
                if (previous.status == "located")
                    require(planner.farthest(previous, option.entry) <= 19.9 + 1e-6 && option.channel == 0,
                            "clear alternatives must cover the whole observed source region");
                else
                    require(planner.guaranteedReception(previous, option.entry) &&
                            option.channel == group.id,
                            "measurement alternatives must target their own channel with guaranteed reception");
                for (Point areaPoint : previous.secondRegion) {
                    if (dist(areaPoint, option.entry) >= 1e-5) continue;
                    double side = cross(unit(previous.obs.front().second),
                                        areaPoint - previous.obs.front().first);
                    lowerAreaBranch = lowerAreaBranch || side < 0;
                    upperAreaBranch = upperAreaBranch || side > 0;
                }
            }
            if (!previous.secondRegion.empty())
                require(lowerAreaBranch && upperAreaBranch,
                        "both disconnected S2 area branches must participate in joint route selection");
        }
        require(taskIds.count(7) && taskIds.count(8) && taskIds.count(9) && groups.size() > 5,
                "one larger window must include S2, refinement, clear and search tasks together");
        for (size_t i = 0; i < planner.search.size(); ++i)
            require(taskIds.count(-1 - static_cast<int>(i)), "every unfinished coverage station must enter the window");
        std::set<int> plannedGroups;
        for (const auto& choice : decision.choices) {
            require(choice.first >= 0 && static_cast<size_t>(choice.first) < groups.size() &&
                    choice.second >= 0 && static_cast<size_t>(choice.second) < groups[choice.first].options.size() &&
                    plannedGroups.insert(choice.first).second,
                    "a route must select at most one valid candidate per task group");
        }
        for (size_t i = 0; i < groups.size(); ++i)
            if (groups[i].id < 0)
                require(plannedGroups.count(static_cast<int>(i)), "optional target gains cannot omit mandatory coverage");
        require(!decision.choices.empty() && decision.visits == static_cast<int>(decision.choices.size()) &&
                decision.choices.front() == std::make_pair(decision.group, decision.option),
                "the returned first action must agree with the reconstructed mixed route");
        Point entry = groups.at(decision.group).options.at(decision.option).entry;
        require(mock.calls.size() > firstCall &&
                (mock.calls[firstCall].first == "/measure" || mock.calls[firstCall].first == "/clear") &&
                dist(requestPoint(mock.calls[firstCall].second), entry) < 1e-8,
                "mixed scheduling must execute the chosen real action at its selected entry");
        require(session.failed == 0 && planner.unifiedGuaranteedMisses == 0 && planner.areaMisses == 0,
                "executing a mixed window must preserve reception and clearing certificates");
        for (const auto& item : planner.targets)
            require(containsSource(item.second, mock.sources.at(item.first).position),
                    "mixed execution must preserve every uncleared source in its observed region");
        timeMatches(session);
        session.exit();
    }
}

#endif
