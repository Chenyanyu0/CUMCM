#include "planner.hpp"
#include "offline_tests/tests.hpp"

void writeJson(const string& path, const Json& value) {
    std::ofstream output(path);
    if (!output) throw std::runtime_error("Cannot open output: " + path);
    output << value.dump(2) << '\n';
    if (!output) throw std::runtime_error("Cannot write output: " + path);
}

int main(int argc, char** argv) {
    try {
#ifdef _WIN32
        // The source and diagnostic labels are UTF-8; keep Windows consoles
        // from decoding those bytes with the legacy system code page.
        SetConsoleOutputCP(CP_UTF8);
        SetConsoleCP(CP_UTF8);
#endif
        string robot, url = "http://127.0.0.1:2026";
        string logPath = "logs/problem4_cpp_" + std::to_string(GetTickCount64()) + ".jsonl";
        string reportPath, exportPath;
        bool test = false, interleave = true, closeApproach = true;
        Json optimizationOptions = Json::object();
        int cases = 100;
        for (int i = 1; i < argc; ++i) {
            string option = argv[i];
            if (option == "--self-test") test = true;
            else if (option == "--no-interleave") interleave = false;
            else if (option == "--no-approach") closeApproach = false;
            else if (option == "--no-route-clear") optimizationOptions["routeClear"] = false;
            else if (option == "--no-unified-choice") optimizationOptions["unifiedChoice"] = false;
            else if (option == "--estimate-candidates") optimizationOptions["estimateCandidates"] = true;
            else if (option == "--unified-choice") optimizationOptions["unifiedChoice"] = true;
            else if (option == "--trial-clear") optimizationOptions["trialClear"] = true;
            else if (option == "--help") {
                std::cout << "problem4.exe --robot-id TEAM [--url http://127.0.0.1:2026] [--log FILE]\n"
                             "problem4.exe --self-test [--cases 100] [--report FILE]\n"
                             "problem4.exe --export-plan FILE\n"
                             "--no-interleave disables en-route probe insertions; station sharing remains enabled.\n"
                             "--no-approach disables certified close-range probes after two bearings.\n"
                             "--no-route-clear disables certified en-route clearing.\n"
                             "--no-unified-choice disables completion-cost comparison of approach and F/G plans.\n"
                             "--estimate-candidates / --unified-choice / --trial-clear enable experimental policies.\n";
                return 0;
            } else if (i + 1 < argc && option == "--robot-id") robot = argv[++i];
            else if (i + 1 < argc && option == "--url") url = argv[++i];
            else if (i + 1 < argc && option == "--log") logPath = argv[++i];
            else if (i + 1 < argc && option == "--report") reportPath = argv[++i];
            else if (i + 1 < argc && option == "--export-plan") exportPath = argv[++i];
            else if (i + 1 < argc && option == "--cases") {
                string text = argv[++i];
                size_t used = 0;
                cases = std::stoi(text, &used);
                if (used != text.size()) throw std::runtime_error("Invalid --cases");
            } else throw std::runtime_error("Unknown or incomplete option: " + option);
        }
        if (!exportPath.empty()) {
            auto mesh = directional_geometry::makeSearchMesh();
            Json points = Json::array();
            std::map<int, Point> destinations;
            for (size_t i = 0; i < mesh.stations.size(); ++i) {
                points.push_back({{"id", i}, {"x", mesh.stations[i].x}, {"y", mesh.stations[i].y}});
                destinations[static_cast<int>(i)] = mesh.stations[i];
            }
            writeJson(exportPath, {{"disk_radius_m", 1800}, {"outer_radius_m", mesh.outerRadius},
                {"max_triangle_edge_m", mesh.outerRadius / 2}, {"stations", points},
                {"triangles", mesh.triangles}, {"initial_route", route({0, 0}, destinations)}});
            std::cout << "Exported 28 triangles and 22 stations to " << exportPath << '\n';
            return 0;
        }
        if (test) return selfTest(cases, reportPath, interleave, closeApproach, optimizationOptions);
        if (robot.empty()) throw std::runtime_error("--robot-id is required; use --self-test offline");
        WinHttpTransport transport(url);
        Session session(transport, robot, logPath);
        Planner planner(session);
        planner.interleave = interleave;
        planner.closeApproach = closeApproach;
        planner.routeClear = optimizationOptions.value("routeClear", planner.routeClear);
        planner.estimateCandidates = optimizationOptions.value("estimateCandidates", planner.estimateCandidates);
        planner.unifiedChoice = optimizationOptions.value("unifiedChoice", planner.unifiedChoice);
        planner.trialClear = optimizationOptions.value("trialClear", planner.trialClear);
        string error;
        try { planner.run(); }
        catch (const std::exception& e) {
            error = e.what();
            if (session.entered && !session.uncertain && nowSeconds() < session.deadline) {
                try { session.exit(); }
                catch (const std::exception& exitError) { error += "; exit: " + string(exitError.what()); }
            }
        }
        Json summary = session.summary();
        summary["planner"] = planner.diagnostics();
        summary["complete"] = planner.complete && error.empty();
        summary["cleared_channels"] = planner.done;
        summary["absent_channels"] = planner.absent;
        summary["search_points_visited"] = planner.scanned;
        summary["error"] = error;
        // Discovery is accounted for separately so the localization/clear
        // average reflects only the work after the scan has finished.
        double localizationClearTime = 0;
        if (summary.contains("phases") && summary["phases"].is_object()) {
            for (auto it = summary["phases"].begin(); it != summary["phases"].end(); ++it)
                if (it.key() != "search" && it.value().contains("virtual_time_s"))
                    localizationClearTime += it.value().at("virtual_time_s").get<double>();
        }
        const int clearedSources = static_cast<int>(planner.done.size());
        const int totalSources = static_cast<int>(planner.targets.size());
        const double clearRatio = totalSources > 0 ?
            static_cast<double>(clearedSources) / totalSources : 0.0;
        const double averageLocalizationClearTime = clearedSources > 0 ?
            localizationClearTime / clearedSources : 0.0;
        summary["cleared_source_count"] = clearedSources;
        summary["total_source_count"] = totalSources;
        summary["localization_clear_total_time_s"] = localizationClearTime;
        summary["cleared_source_ratio"] = clearRatio;
        summary["average_localization_clear_time_s"] = averageLocalizationClearTime;
        session.record({{"kind", "summary"}, {"summary", summary}});
        writeJson(logPath + ".summary.json", summary);
        std::cout << summary.dump(2) << "\n"
                  << "被清除干扰源个数: " << clearedSources << '\n'
                  << "干扰源总数: " << totalSources << '\n'
                  << "定位清除总时间: " << localizationClearTime << " 秒\n"
                  << "被清除干扰源个数的比例: " << clearRatio * 100.0 << "%\n"
                  << "平均定位清除时间: " << averageLocalizationClearTime << " 秒\n"
                  << "log: " << logPath << '\n';
        if (!error.empty()) { std::cerr << "ERROR: " << error << '\n'; return 2; }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 2;
    }
}
