#include "../../planner.hpp"
#include "../tests.hpp"

namespace {
Mock scenarioMock(const Json& scenario) {
    Mock mock;
    mock.seed = scenario.at("seed");
    mock.mode = scenario.at("mode");
    if (mock.mode < 0 || mock.mode > 4) throw std::runtime_error("Unknown error mode");
    for (const auto& source : scenario.at("sources")) {
        int channel = source.at("channel");
        Point position{source.at("x"), source.at("y")};
        double radius = source.at("radius");
        double direction = source.at("direction");
        if (channel < 1 || channel > 20 || mock.sources.count(channel) ||
            !std::isfinite(position.x) || !std::isfinite(position.y) || norm(position) > 1800.000001 ||
            !std::isfinite(radius) || radius < 1000 || radius > 1500 || !std::isfinite(direction))
            throw std::runtime_error("Invalid benchmark source");
        mock.sources[channel] = {position, radius, source.at("directional").get<bool>(), direction};
    }
    if (mock.sources.size() < 10 || mock.sources.size() > 16)
        throw std::runtime_error("Benchmark requires 10..16 sources");
    return mock;
}

Json run(const Json& request) {
    double started = nowSeconds();
    const Json& scenario = request.at("scenario");
    Json options = request.value("options", Json::object());
    Mock mock = scenarioMock(scenario);
    Session session(mock, "P4-OPTIMIZATION-OFFLINE", "");
    Planner planner(session, true);
    planner.routeClear = options.value("routeClear", false);
    planner.estimateCandidates = options.value("estimateCandidates", false);
    planner.unifiedChoice = options.value("unifiedChoice", false);
    planner.trialClear = options.value("trialClear", false);
    mock.beforeAction = [&]() { checkTargetContainment(planner, mock); };
    string error;
    try {
        planner.run();
        checkTargetContainment(planner, mock);
        checkOrientations(planner, mock);
        require(planner.complete && mock.cleared.size() == mock.sources.size(), "all sources cleared");
        require(mock.exited, "explicit exit after completion");
        require(session.failed == planner.trialFailures, "only optional trial clears may fail");
        require(planner.approachMisses == 0, "certified close-approach probe receives its source");
        require(std::abs(session.summary()["time_accounting_error_s"].get<double>()) < .001,
                "virtual clock agrees with actions");
        double phaseTime = 0;
        for (const auto& phase : session.phases) phaseTime += phase.second.virtualTime;
        require(std::abs(phaseTime - session.vt) < 1e-5, "phase times partition total time");
        require(planner.done.size() + planner.absent.size() == 20, "every channel accounted for");
        for (int channel : planner.absent)
            require(!mock.sources.count(channel), "absent-channel certificate is sound");
    } catch (const std::exception& exception) {
        error = exception.what();
        mock.beforeAction = nullptr;
        if (session.entered && !session.uncertain) {
            try { session.exit(); } catch (...) {}
        }
    }
    Json result = session.summary();
    result["sources"] = mock.sources.size();
    result["complete"] = error.empty() && mock.cleared.size() == mock.sources.size();
    result["cleared_channels"] = mock.cleared;
    result["remaining_channels"] = Json::array();
    for (const auto& source : mock.sources)
        if (!mock.cleared.count(source.first)) result["remaining_channels"].push_back(source.first);
    result["exited"] = mock.exited;
    result["error"] = error;
    Json diagnostics = planner.diagnostics();
    for (const string& field : {"approach_plans", "detour_plans", "transit_clear_plans", "trial_clear_plans"})
        diagnostics.erase(field);
    result["planner"] = diagnostics;
    result["search_points_visited"] = planner.scanned;
    result["host_time_s"] = nowSeconds() - started;
    return result;
}
}

int main() {
    string line;
    while (std::getline(std::cin, line)) {
        try {
            Json request = Json::parse(line);
            Json response = {{"id", request.at("id")}, {"result", run(request)}};
            std::cout << response.dump() << std::endl;
        } catch (const std::exception& error) {
            std::cout << Json({{"bridge_error", error.what()}}).dump() << std::endl;
        }
    }
}
