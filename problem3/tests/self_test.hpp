#ifndef PROBLEM3_TESTS_SELF_TEST_HPP
#define PROBLEM3_TESTS_SELF_TEST_HPP

// Offline test entry point; never constructs the HTTP transport.
#include "mock_transport.hpp"
#include "test_support.hpp"
#include "geometry_tests.hpp"

Mock makeCase(int seed) {
    Mock mock;
    mock.seed = seed;
    mock.mode = seed % 4;
    std::mt19937 random(seed + 912);
    std::uniform_real_distribution<double> uniform(0, 1);
    vector<int> channels;
    for (int k = 1; k <= 20; ++k) channels.push_back(k);
    std::shuffle(channels.begin(), channels.end(), random);
    int count = 10 + seed % 7;
    for (int k = 0; k < count; ++k) {
        double angle = uniform(random) * 360;
        double radius = 1800 * std::sqrt(uniform(random));
        Point source = unit(angle) * radius;
        if (seed % 8 == 0) source = unit(30 + k * 60.0) * 1800;
        if (seed % 8 == 1 && k == 0) source = {0, 0};
        mock.sources[channels[k]] = {source, seed % 3 == 0 ? 1000 : 1000 + 500 * uniform(random)};
    }
    return mock;
}

Json testCase(Mock& mock) {
    Session session(mock, "TEST", "");
    Planner planner(session, true);
    planner.run();
    require(planner.complete && mock.cleared.size() == mock.sources.size(), "all sources cleared");
    require(mock.exited, "explicit /exit");
    require(std::abs(session.summary()["time_accounting_error_s"].get<double>()) < .001, "clock matches actions");
    require(session.failed == planner.probeFailures, "all failures belong to bounded speculative clears");
    require(planner.probeAttempts == planner.probeSuccesses + planner.probeFailures &&
            planner.probeAttempts <= static_cast<int>(planner.targets.size()), "one speculative clear per target");
    require(planner.unifiedGuaranteedMisses == 0, "guaranteed unified candidate reception");
    require(planner.areaMisses == 0, "guaranteed area reception");
    require(planner.oneReadFailures == 0, "certified one-reading candidates reach clearing precision");
    require(planner.areaCreated == planner.areaRetired, "all first-observation area tasks retired");
    require(planner.scanned == 7 || planner.targets.size() == 16, "coverage before declaring absent channels");
    require(std::set<int>(planner.stationVisitOrder.begin(), planner.stationVisitOrder.end()).size() ==
            planner.stationVisitOrder.size(), "each search station visited at most once");
    require(planner.stationVisitOrder.size() == static_cast<size_t>(planner.scanned), "completed station count");
    double phaseTime = 0;
    for (const auto& phase : session.phases) phaseTime += phase.second.virtualTime;
    require(std::abs(phaseTime - session.vt) < 1e-5, "phase times partition virtual time");
    require(session.phases["shared"].distance < 1e-5, "shared readings add no travel");
    for (int channel : planner.absent) require(!mock.sources.count(channel), "absent-channel certificate");
    for (const auto& item : planner.targets) {
        const auto& t = item.second;
        require(t.failedClearPoints.size() <= 1 && (t.failedClearPoints.empty() || t.probeAttempted),
                "speculative failures retain a permanent per-target record");
        require(t.discoveredTime <= t.locatedTime && t.locatedTime <= t.clearedTime,
                "target event timestamps are ordered");
        Point source = mock.sources.at(item.first).position;
        for (size_t i = 0; i < t.region.size(); ++i) {
            Point a = t.region[i], edge = t.region[(i + 1) % t.region.size()] - a;
            require(cross(edge, source - a) >= -1e-5 * std::max(1.0, norm(edge)),
                    "real source remains inside the nested observation region");
        }
    }
    for (const auto& history : planner.measuredPoints) {
        for (size_t i = 0; i < history.second.size(); ++i)
            for (size_t j = i + 1; j < history.second.size(); ++j)
                require(dist(history.second[i], history.second[j]) > 1e-5, "no repeated channel/point reading");
    }
    Json result = session.summary();
    result["shared_measurements"] = planner.sharedMeasures;
    result["shared_localizations"] = planner.sharedLocations;
    result["shared_no_signal"] = planner.sharedMisses;
    result["homing_moves"] = planner.homingMoves;
    result["center_measurements"] = planner.centerMeasures;
    result["contour_candidates"] = planner.contourCandidates;
    result["area_tasks_created"] = planner.areaCreated;
    result["area_measurements"] = planner.areaMeasures;
    result["unified_dispatches"] = planner.unifiedDispatches;
    result["unified_search_choices"] = planner.unifiedSearchChoices;
    result["unified_local_choices"] = planner.unifiedLocalChoices;
    result["planner"] = planner.diagnostics();
    return result;
}

double quantile(vector<double> values, double fraction) {
    std::sort(values.begin(), values.end());
    return values[static_cast<size_t>(std::ceil(fraction * values.size())) - 1];
}

int selfTest(int cases, const string& referencePath, const string& reportPath, int firstSeed = 0) {
    geometryTests();
    int total = 0;
    double started = nowSeconds();
    Json reference;
    std::map<int, Json> referenceRows;
    bool compare = !referencePath.empty();
    if (compare) {
        std::ifstream input(referencePath);
        if (!input) throw std::runtime_error("Cannot read reference report: " + referencePath);
        input >> reference;
        if (!reference.contains("case_results") || !reference["case_results"].is_array() ||
            reference["case_results"].size() < static_cast<size_t>(cases))
            throw std::runtime_error("Reference report does not contain enough cases");
        for (const Json& row : reference["case_results"]) {
            if (!row.contains("seed") || !row["seed"].is_number_integer())
                throw std::runtime_error("Reference report has a missing or invalid seed");
            if (!referenceRows.emplace(row["seed"].get<int>(), row).second)
                throw std::runtime_error("Reference report has duplicate seeds");
        }
        for (int seed = firstSeed; seed < firstSeed + cases; ++seed) {
            if (!referenceRows.count(seed)) throw std::runtime_error("Reference report is missing seed " + std::to_string(seed));
            const Json& row = referenceRows.at(seed);
            if (row.at("seed") != seed || row.at("sources") != 10 + seed % 7 || !row.contains("optimized"))
                throw std::runtime_error("Reference report has incompatible seeds or source counts");
            double time = row.at("optimized").at("virtual_time_s").get<double>();
            if (!std::isfinite(time) || time <= 0) throw std::runtime_error("Invalid reference time");
        }
    }
    vector<Json> results;
    Json report = {{"cases", cases}, {"seed_start", firstSeed}, {"case_results", Json::array()},
                   {"planner_version", PLANNER_VERSION}, {"case_generator", "synthetic-v1"},
                   {"description", "Local synthetic cases, not official simulator scores"}};
    report["area_model"] = {{"threshold_m2", area_region::threshold()},
        {"grid_spacing_m", area_region::spacing()}, {"template_points", area_region::samples().size()},
        {"quadrature", {16, 32, 16}}, {"per_reading_prediction_margin_s", 8}};
    report["window_model"] = {{"max_groups", 10}, {"max_options_per_group", 10},
        {"search_target_groups", 4}, {"local_target_groups", 7}, {"max_actions_between_stations", 8},
        {"mandatory_pending_stations", true}, {"execution", "first real action then replan"}};
    report["one_read_model"] = {{"vertex_distance_limit_m", one_read_region::limit()},
        {"center_retained_after_two_bearings", true}, {"max_region_candidates", 8},
        {"certification", "all source-region vertices within the measurement radius limit"}};
    report["probe_clear_model"] = {{"max_attempts_per_target", 1}, {"max_mec_radius_m", 80},
        {"min_area_coverage", .4}, {"coverage_is_calibrated_probability", false},
        {"position", "already selected next measurement point"}, {"failure", "measure at the same point"}};
    report["estimate_model"] = {{"all_bearings", true}, {"constrained_to_actual_region", true},
        {"supplies_clear_certificate", false}, {"max_iterations", 8}};
    report["clear_region_model"] = {{"vertex_disk_radius_m", clear_region::limit()},
        {"full_disk_intersection", true}, {"max_candidates", 10}};
    if (compare) {
        report["reference_report"] = referencePath;
        report["reference_version"] = reference.value("planner_version", string("optimized-v1"));
    }
    for (int seed = firstSeed; seed < firstSeed + cases; ++seed) {
        Mock initial = makeCase(seed);
        total += static_cast<int>(initial.sources.size());
        Json row = {{"seed", seed}, {"sources", initial.sources.size()}};
        try { results.push_back(testCase(initial)); }
        catch (const std::exception& e) {
            throw std::runtime_error("seed " + std::to_string(seed) + ": " + e.what());
        }
        const Json& metrics = results.back();
        row["optimized"] = {{"virtual_time_s", metrics["virtual_time_s"]}, {"distance_m", metrics["distance_m"]},
                            {"measurements", metrics["measurements"]}, {"average_time_s", metrics["average_time_s"]},
                            {"failed_clears", metrics["failed_clears"]}, {"switches", metrics["switches"]}};
        row["planner"] = metrics["planner"];
        if (compare) row["reference"] = referenceRows.at(seed)["optimized"];
        report["case_results"].push_back(row);
        if ((seed - firstSeed + 1) % 100 == 0)
            std::cout << "tested " << seed - firstSeed + 1 << "/" << cases << std::endl;
    }
    double totalTime = 0, distance = 0, measurementCount = 0, caseAverage = 0;
    int shared = 0, localized = 0, missed = 0, improved = 0, worsened = 0;
    int centerReadings = 0, contourPoints = 0;
    int areaTaskCount = 0, areaReadingCount = 0;
    int unifiedCount = 0, unifiedSearchCount = 0, unifiedLocalCount = 0;
    std::map<string, int> unifiedMetrics;
    double baselineTime = 0, largestRegression = 0;
    vector<double> times, averages;
    std::map<string, double> phaseTimes, phaseDistances;
    for (int i = 0; i < cases; ++i) {
        const Json& metrics = results[i];
        double time = metrics["virtual_time_s"], average = metrics["average_time_s"];
        totalTime += time; distance += metrics["distance_m"].get<double>();
        measurementCount += metrics["measurements"].get<int>(); caseAverage += average;
        times.push_back(time); averages.push_back(average);
        shared += metrics["shared_measurements"].get<int>();
        localized += metrics["shared_localizations"].get<int>();
        missed += metrics["shared_no_signal"].get<int>();
        centerReadings += metrics["center_measurements"].get<int>();
        contourPoints += metrics.value("contour_candidates", 0);
        areaTaskCount += metrics.value("area_tasks_created", 0);
        areaReadingCount += metrics.value("area_measurements", 0);
        unifiedCount += metrics.value("unified_dispatches", 0);
        unifiedSearchCount += metrics.value("unified_search_choices", 0);
        unifiedLocalCount += metrics.value("unified_local_choices", 0);
        for (const char* key : {"unified_mixed_windows", "unified_refinements", "unified_clears_during_search",
                "unified_guaranteed_misses", "unified_s2_windows", "unified_region_windows", "unified_multi_before_search",
                "one_read_regions", "one_read_candidates", "one_read_measurements", "one_read_off_center", "one_read_failures",
                "probe_attempts", "probe_successes", "probe_failures"})
            unifiedMetrics[key] += metrics["planner"].value(key, 0);
        unifiedMetrics["unified_max_window"] = std::max(unifiedMetrics["unified_max_window"],
            metrics["planner"].value("unified_max_window", 0));
        for (auto phase = metrics["phases"].begin(); phase != metrics["phases"].end(); ++phase) {
            phaseTimes[phase.key()] += phase.value()["virtual_time_s"].get<double>() / cases;
            phaseDistances[phase.key()] += phase.value()["distance_m"].get<double>() / cases;
        }
        if (compare) {
            double base = referenceRows.at(firstSeed + i)["optimized"]["virtual_time_s"];
            baselineTime += base;
            if (time < base - 1e-5) ++improved;
            if (time > base + 1e-5) ++worsened;
            largestRegression = std::max(largestRegression, time - base);
        }
    }
    Json aggregate = {{"cases_passed", cases}, {"sources_cleared", total},
        {"weighted_average_time_s", totalTime / total}, {"mean_case_average_time_s", caseAverage / cases},
        {"mean_virtual_time_s", totalTime / cases}, {"p95_virtual_time_s", quantile(times, .95)},
        {"worst_case_average_time_s", quantile(averages, 1)}, {"mean_distance_m", distance / cases},
        {"mean_measurements", measurementCount / cases}, {"shared_measurements", shared},
        {"shared_localizations", localized}, {"shared_no_signal", missed}};
    aggregate["center_measurements"] = centerReadings;
    aggregate["contour_candidates"] = contourPoints;
    aggregate["area_tasks_created"] = areaTaskCount;
    aggregate["area_measurements"] = areaReadingCount;
    aggregate["unified_dispatches"] = unifiedCount;
    aggregate["unified_search_choices"] = unifiedSearchCount;
    aggregate["unified_local_choices"] = unifiedLocalCount;
    for (const auto& metric : unifiedMetrics) aggregate[metric.first] = metric.second;
    aggregate["mean_failed_clears"] = unifiedMetrics["probe_failures"] / static_cast<double>(cases);
    aggregate["max_virtual_time_s"] = quantile(times, 1);
    aggregate["mean_phase_time_s"] = phaseTimes;
    aggregate["mean_phase_distance_m"] = phaseDistances;
    if (compare) {
        aggregate["reference_mean_virtual_time_s"] = baselineTime / cases;
        aggregate["total_time_reduction_percent"] = 100 * (baselineTime - totalTime) / baselineTime;
        aggregate["improved_cases"] = improved;
        aggregate["worsened_cases"] = worsened;
        aggregate["largest_regression_s"] = largestRegression;
    }
    report["strategies"]["optimized"] = aggregate;
    std::cout << "optimized: " << aggregate.dump() << '\n';
    report["total_sources"] = total;
    report["completion_summary"] = completionMetrics(total, total, totalTime);
    report["real_test_time_s"] = nowSeconds() - started;
    if (!reportPath.empty()) {
        std::ofstream output(reportPath);
        if (!output) throw std::runtime_error("Cannot write report: " + reportPath);
        output << report.dump(2) << '\n';
        if (!output) throw std::runtime_error("Failed to write report: " + reportPath);
    }
    std::cout << "self-test OK: " << cases << "/" << cases << " cases, " << total << "/" << total
              << " sources cleared; real_test_time_s=" << nowSeconds() - started << '\n';
    printCompletionMetrics(report["completion_summary"], true);
    return 0;
}

#endif
