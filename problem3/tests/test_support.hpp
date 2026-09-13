#ifndef PROBLEM3_TESTS_TEST_SUPPORT_HPP
#define PROBLEM3_TESTS_TEST_SUPPORT_HPP

// Test assertions and the exact point-route reference solver.
inline void require(bool value, const string& message) {
    if (!value) throw std::runtime_error("Self-test: " + message);
}

vector<int> exactRoute(Point start, const std::map<int, Point>& destinations) {
    if (destinations.empty()) return {};
    if (destinations.size() > 16)
        throw std::invalid_argument("Exact route supports at most 16 destinations");
    vector<int> channels;
    vector<Point> points;
    for (const auto& item : destinations) {
        channels.push_back(item.first);
        points.push_back(item.second);
    }
    const int count = static_cast<int>(points.size());
    const int states = 1 << count;
    vector<double> distances(count * count);
    for (int i = 0; i < count; ++i)
        for (int j = 0; j < count; ++j)
            distances[i * count + j] = dist(points[i], points[j]);
    vector<double> costs(states * count, std::numeric_limits<double>::infinity());
    vector<signed char> previous(states * count, -1);
    for (int i = 0; i < count; ++i)
        costs[(1 << i) * count + i] = dist(start, points[i]);
    // Each state visits exactly mask and ends at last; the endpoint is unrestricted.
    for (int mask = 1; mask < states; ++mask) {
        for (int last = 0; last < count; ++last) {
            if (!(mask & (1 << last))) continue;
            int prefix = mask ^ (1 << last);
            if (!prefix) continue;
            int index = mask * count + last;
            for (int before = 0; before < count; ++before) {
                if (!(prefix & (1 << before))) continue;
                double candidate = costs[prefix * count + before] + distances[before * count + last];
                if (candidate < costs[index]) {
                    costs[index] = candidate;
                    previous[index] = static_cast<signed char>(before);
                }
            }
        }
    }
    int mask = states - 1, last = 0;
    for (int i = 1; i < count; ++i)
        if (costs[mask * count + i] < costs[mask * count + last]) last = i;
    vector<int> order;
    while (mask) {
        order.push_back(channels[last]);
        int before = previous[mask * count + last];
        mask ^= 1 << last;
        last = before;
    }
    std::reverse(order.begin(), order.end());
    return order;
}

#endif
