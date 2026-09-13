#ifndef PROBLEM3_TESTS_MOCK_TRANSPORT_HPP
#define PROBLEM3_TESTS_MOCK_TRANSPORT_HPP

// Test-only transport; included after the production Session and Planner.
// This transport exposes only documented observations to the planner.
struct Mock : ITransport {
    struct Source { Point position; double radius; };
    std::map<int, Source> sources;
    std::map<string, pair<string, string>> requests;
    Point position;
    int current = 1, seed = 0, mode = 0;
    double time = 0;
    bool entered = false, exited = false;
    std::set<int> cleared;

    string post(const string& path, const string& body, double) override {
        Json request = Json::parse(body);
        string id = request.at("request_id");
        string signature = path + body;
        auto old = requests.find(id);
        if (old != requests.end()) {
            if (old->second.first != signature) throw std::runtime_error("Duplicate ID changed action");
            return old->second.second;
        }
        Json response = {{"accepted", true}, {"real_timestamp_ms", 0}};
        if (path == "/enter") {
            if (entered || exited) throw std::runtime_error("Duplicate enter");
            entered = true;
            response["remaining_real_duration_s"] = 1200;
            response["max_virtual_duration_s"] = 360000;
        } else if (!entered || exited) throw std::runtime_error("Inactive mock session");
        else if (path == "/exit") { exited = true; response["exit_reason"] = "user_exit"; }
        else {
            int channel = request.at("channel");
            Point next{request.at("position").at("x"), request.at("position").at("y")};
            time += dist(position, next) / 5;
            position = next;
            auto source = sources.find(channel);
            double distance = source == sources.end() || cleared.count(channel) ? 1e100 :
                dist(position, source->second.position);
            if (path == "/measure") {
                time += 5 + (channel != current);
                current = channel;
                if (distance <= 5) response["measure_result"] = "near";
                else if (source != sources.end() && distance <= source->second.radius) {
                    response["measure_result"] = "direction";
                    double error = mode == 1 ? 1 : mode == 2 ? -1 : mode == 3 ? 0 :
                        std::sin(position.x * .013 + position.y * .017 + channel * 3.71 + seed);
                    double angle = std::fmod(bearing(position, source->second.position) + error + 360, 360);
                    response["svd_deg"] = std::fmod(std::round(angle * 100) / 100, 360);
                } else response["measure_result"] = "no_signal";
            } else if (path == "/clear") {
                bool success = distance <= 20;
                time += success ? 5 : 3;
                if (success) cleared.insert(channel);
                response["clear_result"] = success ? "success" : "no_target_in_range";
            } else throw std::runtime_error("Unknown path");
        }
        response["virtual_time_s"] = std::round(time * 1e6) / 1e6;
        string output = response.dump(2); // Whitespace and key ordering must not matter.
        requests[id] = {signature, output};
        return output;
    }
};

#endif
