#ifndef PROBLEM3_NEIGHBORHOOD_ROUTE_HPP
#define PROBLEM3_NEIGHBORHOOD_ROUTE_HPP

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

struct RouteOption {
    Point entry, exit;
    int channel = 0;
    double service = 0;
    int endChannel = 0;
};

struct RouteGroup {
    int id = 0;
    std::vector<RouteOption> options;
};

struct RouteDecision {
    int group = -1, option = -1;
    double cost = std::numeric_limits<double>::infinity();
    int visits = 0;
    std::vector<std::pair<int, int>> choices;
    // Distance between action endpoints, including an optional final anchor.
    // Internal entry-to-exit travel, if any, belongs to the option's service.
    double distance = std::numeric_limits<double>::infinity();
};

inline RouteDecision solveNeighborhoodRoute(Point start, int current,
        const std::vector<RouteGroup>& groups, bool optional,
        const Point* anchor = nullptr, int onwardChannel = 0,
        int requiredMask = -1) {
    if (groups.size() > 10 || current < 1 || current > 20 ||
        onwardChannel < 0 || onwardChannel > 20)
        throw std::invalid_argument("Invalid neighborhood route dimensions or channel");
    auto finitePoint = [](Point p) { return std::isfinite(p.x) && std::isfinite(p.y); };
    if (!finitePoint(start) || (anchor && !finitePoint(*anchor)))
        throw std::invalid_argument("Nonfinite neighborhood route endpoint");
    if (requiredMask < -1 || (requiredMask >= 0 &&
        requiredMask >= (1 << static_cast<int>(groups.size()))))
        throw std::invalid_argument("Invalid neighborhood route required mask");
    struct OptionIndex { int group, option; };
    std::vector<OptionIndex> indices;
    for (size_t i = 0; i < groups.size(); ++i) {
        if (groups[i].options.size() > 10)
            throw std::invalid_argument("Neighborhood route supports at most ten options per group");
        for (size_t j = 0; j < groups[i].options.size(); ++j) {
            const RouteOption& option = groups[i].options[j];
            if (option.channel < 0 || option.channel > 20 ||
                option.endChannel < 0 || option.endChannel > 20 || !std::isfinite(option.service) ||
                !finitePoint(option.entry) || !finitePoint(option.exit))
                throw std::invalid_argument("Invalid neighborhood route option");
            indices.push_back({static_cast<int>(i), static_cast<int>(j)});
        }
    }
    auto at = [&](int index) -> const RouteOption& {
        const OptionIndex& selected = indices[index];
        return groups[selected.group].options[selected.option];
    };
    auto tailCost = [&](Point from, int channel) {
        return anchor ? dist(from, *anchor) / 5 +
            (onwardChannel != 0 && onwardChannel != channel ? 1 : 0) : 0.0;
    };
    const int states = 1 << groups.size();
    const int mandatory = optional ? std::max(0, requiredMask) : states - 1;
    RouteDecision result;
    if (mandatory == 0) {
        result.cost = tailCost(start, current);
        result.distance = anchor ? dist(start, *anchor) : 0;
    }
    const int optionCount = static_cast<int>(indices.size());
    if (!optionCount) return result;
    struct State {
        double cost = std::numeric_limits<double>::infinity();
        int previous = -1;
    };
    // A clear action keeps the previous channel, so its endpoint alone cannot
    // identify future switch costs. Retain the channel as part of the state.
    std::vector<State> table(states * optionCount * 21);
    auto cellIndex = [&](int mask, int last, int channel) {
        return (mask * optionCount + last) * 21 + channel;
    };
    std::vector<std::vector<int>> endingChannels(optionCount);
    std::vector<int> possibleChannels{current};
    for (int i = 0; i < optionCount; ++i) {
        const RouteOption& option = at(i);
        const int fixed = option.endChannel != 0 ? option.endChannel : option.channel;
        if (fixed != 0 && std::find(possibleChannels.begin(), possibleChannels.end(), fixed) ==
                possibleChannels.end()) possibleChannels.push_back(fixed);
    }
    std::vector<double> edges(optionCount * optionCount);
    for (int i = 0; i < optionCount; ++i) {
        const RouteOption& option = at(i);
        int actionChannel = option.channel == 0 ? current : option.channel;
        int channel = option.endChannel != 0 ? option.endChannel : actionChannel;
        const int fixed = option.endChannel != 0 ? option.endChannel : option.channel;
        endingChannels[i] = fixed == 0 ? possibleChannels : std::vector<int>{fixed};
        State& initial = table[cellIndex(1 << indices[i].group, i, channel)];
        initial.cost = dist(start, option.entry) / 5 + option.service +
                       (actionChannel != current);
        for (int j = 0; j < optionCount; ++j)
            edges[i * optionCount + j] = dist(option.exit, at(j).entry) / 5;
    }
    int bestState = -1;
    for (int mask = 1; mask < states; ++mask) {
        int visits = 0;
        for (int bits = mask; bits; bits &= bits - 1) ++visits;
        for (int last = 0; last < optionCount; ++last) {
            if (!(mask & (1 << indices[last].group))) continue;
            for (int channel : endingChannels[last]) {
                const int stateIndex = cellIndex(mask, last, channel);
                const State state = table[stateIndex];
                if (!std::isfinite(state.cost)) continue;
                if ((mask & mandatory) == mandatory) {
                    const double cost = state.cost + tailCost(at(last).exit, channel);
                    if (cost < result.cost - 1e-9 ||
                        (std::abs(cost - result.cost) <= 1e-9 && visits < result.visits)) {
                        result.cost = cost;
                        result.visits = visits;
                        bestState = stateIndex;
                    }
                }
                for (int next = 0; next < optionCount; ++next) {
                    const int bit = 1 << indices[next].group;
                    if (mask & bit) continue;
                    const RouteOption& option = at(next);
                    const int actionChannel = option.channel == 0 ? channel : option.channel;
                    const int nextChannel = option.endChannel != 0 ? option.endChannel : actionChannel;
                    const double cost = state.cost + edges[last * optionCount + next] +
                                        option.service + (actionChannel != channel);
                    State& destination = table[cellIndex(mask | bit, next, nextChannel)];
                    if (cost < destination.cost) {
                        destination.cost = cost;
                        destination.previous = stateIndex;
                    }
                }
            }
        }
    }
    if (bestState >= 0) {
        for (int index = bestState; index >= 0; index = table[index].previous) {
            const OptionIndex& selected = indices[(index / 21) % optionCount];
            result.choices.emplace_back(selected.group, selected.option);
        }
        std::reverse(result.choices.begin(), result.choices.end());
        result.group = result.choices.front().first;
        result.option = result.choices.front().second;
        result.distance = 0;
        Point from = start;
        for (const auto& selected : result.choices) {
            const RouteOption& action = groups[selected.first].options[selected.second];
            result.distance += dist(from, action.entry);
            from = action.exit;
        }
        if (anchor) result.distance += dist(from, *anchor);
    }
    return result;
}

inline RouteDecision solveBudgetedNeighborhoodRoute(Point start, int current,
        const std::vector<RouteGroup>& groups, Point anchor, int onwardChannel,
        double extraDistanceBudget) {
    auto finitePoint = [](Point p) { return std::isfinite(p.x) && std::isfinite(p.y); };
    if (groups.size() > 5 || current < 1 || current > 20 || onwardChannel < 0 ||
        onwardChannel > 20 || !std::isfinite(extraDistanceBudget) || extraDistanceBudget < 0 ||
        !finitePoint(start) || !finitePoint(anchor))
        throw std::invalid_argument("Invalid budgeted neighborhood route input");
    struct IndexedOption {
        int group, option;
        const RouteOption* value;
    };
    std::vector<IndexedOption> options;
    for (size_t i = 0; i < groups.size(); ++i) {
        if (groups[i].options.size() > 4)
            throw std::invalid_argument("Budgeted route supports at most four options per group");
        for (size_t j = 0; j < groups[i].options.size(); ++j) {
            const RouteOption& option = groups[i].options[j];
            if (option.channel < 0 || option.channel > 20 ||
                option.endChannel < 0 || option.endChannel > 20 || !std::isfinite(option.service) ||
                !finitePoint(option.entry) || !finitePoint(option.exit) ||
                dist(option.entry, option.exit) > 1e-7)
                throw std::invalid_argument("Budgeted route options must have matching entry and exit");
            options.push_back({static_cast<int>(i), static_cast<int>(j), &option});
        }
    }
    const int count = static_cast<int>(options.size());
    const double direct = dist(start, anchor);
    const double maximum = direct + extraDistanceBudget;
    std::vector<double> fromStart(count), toAnchor(count), between(count * count);
    for (int i = 0; i < count; ++i) {
        fromStart[i] = dist(start, options[i].value->entry);
        toAnchor[i] = dist(options[i].value->exit, anchor);
        for (int j = 0; j < count; ++j)
            between[i * count + j] = dist(options[i].value->exit, options[j].value->entry);
    }
    RouteDecision result;
    result.cost = direct / 5 + (onwardChannel != 0 && onwardChannel != current);
    result.distance = direct;
    // Short routes permit exact enumeration without collapsing paths that have
    // different remaining distance budgets into one cheapest-cost DP state.
    std::function<void(int, int, int, double, double, int, int)> visit;
    std::vector<std::pair<int, int>> path;
    visit = [&](int mask, int last, int channel, double travelled,
                double service, int first, int visits) {
        for (int next = 0; next < count; ++next) {
            const IndexedOption& index = options[next];
            if (mask & (1 << index.group)) continue;
            const RouteOption& option = *index.value;
            const double nextTravel = travelled +
                (last < 0 ? fromStart[next] : between[last * count + next]);
            const double totalDistance = nextTravel + toAnchor[next];
            if (totalDistance > maximum + 1e-7) continue;
            const int actionChannel = option.channel == 0 ? channel : option.channel;
            const int nextChannel = option.endChannel == 0 ? actionChannel : option.endChannel;
            const double nextService = service + option.service + (actionChannel != channel);
            const double total = totalDistance / 5 + nextService +
                (onwardChannel != 0 && onwardChannel != nextChannel);
            const int firstIndex = first < 0 ? next : first;
            path.emplace_back(index.group, index.option);
            if (total < result.cost - 1e-9 ||
                (std::abs(total - result.cost) <= 1e-9 && visits + 1 < result.visits)) {
                result.group = options[firstIndex].group;
                result.option = options[firstIndex].option;
                result.cost = total;
                result.visits = visits + 1;
                result.choices = path;
                result.distance = totalDistance;
            }
            visit(mask | (1 << index.group), next, nextChannel, nextTravel,
                  nextService, firstIndex, visits + 1);
            path.pop_back();
        }
    };
    visit(0, -1, current, 0, 0, -1, 0);
    return result;
}

#endif
