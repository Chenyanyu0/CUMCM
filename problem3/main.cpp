// B棰橀棶棰?鏈哄櫒鐙楃▼搴忥紝C++14 / Windows銆?// 缂栬瘧锛歡++ -std=c++14 -O2 -Wall -Wextra -pedantic main.cpp -o problem3.exe -lwinhttp
#define NOMINMAX
#define _WIN32_WINNT 0x0600
#include <windows.h>
#include <winhttp.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <sstream>
#include <set>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <chrono>
#include "vendor/json.hpp"

using std::string; using std::vector; using std::pair;
const double PI=std::acos(-1.0), ERR=1.0050001, EPS=1e-8;
const char* const PLANNER_VERSION = "bounded-estimate-v11";
struct Point { double x=0,y=0; Point(){} Point(double a,double b):x(a),y(b){} };
Point operator+(Point a,Point b){return {a.x+b.x,a.y+b.y};}
Point operator-(Point a,Point b){return {a.x-b.x,a.y-b.y};}
Point operator*(Point a,double k){return {a.x*k,a.y*k};}
double dot(Point a,Point b){return a.x*b.x+a.y*b.y;}
double cross(Point a,Point b){return a.x*b.y-a.y*b.x;}
double norm(Point a){return std::hypot(a.x,a.y);}
double dist(Point a,Point b){return norm(a-b);}
Point unit(double degrees){double a=degrees*PI/180;return {std::cos(a),std::sin(a)};}
double bearing(Point a,Point b){double z=std::atan2(b.y-a.y,b.x-a.x)*180/PI;return z<0?z+360:z;}
using Poly=vector<Point>;
#include "area_region.hpp"
#include "neighborhood_route.hpp"

Poly clip(const Poly& in,Point anchor,Point normal){
    Poly out; if(in.empty()) return out; Point prev=in.back(); double pv=dot(prev-anchor,normal);
    for(Point cur:in){double cv=dot(cur-anchor,normal); if((pv>=0)!=(cv>=0)){
        double t=pv/(pv-cv); out.push_back(prev+(cur-prev)*t); } if(cv>=0) out.push_back(cur); prev=cur;pv=cv;}
    Poly clean; for(Point p:out) if(clean.empty()||dist(p,clean.back())>EPS) clean.push_back(p);
    if(clean.size()>1&&dist(clean.front(),clean.back())<=EPS) clean.pop_back();
    return clean;
}
Poly disk(Poly p,Point c,double r,int n=64){
    for(int i=0;i<n&&!p.empty();++i){Point u=unit(i*360.0/n);p=clip(p,c+u*(r+EPS),u*-1.0);} return p;
}
Poly outerDisk(Point c,double r,int n=96){
    double R=(r+EPS)/std::cos(PI/n); Poly p; for(int i=0;i<n;++i)p.push_back(c+unit((i+.5)*360/n)*R); return p;
}
Poly wedge(Poly p,Point s,double angle){
    Point lo=unit(angle-ERR),hi=unit(angle+ERR);
    p=clip(p,s-Point(-lo.y,lo.x)*EPS,Point(-lo.y,lo.x));
    p=clip(p,s-Point(hi.y,-hi.x)*EPS,Point(hi.y,-hi.x)); return p;
}
struct Circle {Point c;double r=0;bool contains(Point p)const{return dist(c,p)<=r+1e-7;}};
Circle diameter(Point a,Point b){return {(a+b)*.5,dist(a,b)*.5};}
bool circum(Point a,Point b,Point c,Circle& out){Point u=b-a,v=c-a;double d=2*cross(u,v);if(std::abs(d)<1e-12)return false;double uu=dot(u,u),vv=dot(v,v);Point q={(uu*v.y-vv*u.y)/d,(u.x*vv-v.x*uu)/d};out={a+q,norm(q)};return true;}
Circle twoBoundary(const Poly& points, size_t count, Point a, Point b) {
    Circle base = diameter(a, b), left, right;
    bool hasLeft = false, hasRight = false;
    Point axis = b - a;
    for (size_t i = 0; i < count; ++i) {
        Point point = points[i];
        if (base.contains(point)) continue;
        double side = cross(axis, point - a);
        Circle circle;
        if (!circum(a, b, point, circle)) continue;
        double offset = cross(axis, circle.c - a);
        if (side > 0 && (!hasLeft || offset > cross(axis, left.c - a))) {
            left = circle; hasLeft = true;
        } else if (side < 0 && (!hasRight || offset < cross(axis, right.c - a))) {
            right = circle; hasRight = true;
        }
    }
    if (!hasLeft && !hasRight) return base;
    if (!hasLeft) return right;
    if (!hasRight) return left;
    return left.r <= right.r ? left : right;
}

Circle mec(const Poly& points) {
    if (points.empty()) throw std::runtime_error("Empty localization region");
    Poly ordered = points;
    std::mt19937 random(1741);
    std::shuffle(ordered.begin(), ordered.end(), random);
    Circle circle{ordered[0], 0};
    for (size_t i = 0; i < ordered.size(); ++i) {
        if (circle.contains(ordered[i])) continue;
        circle = {ordered[i], 0};
        for (size_t j = 0; j < i; ++j) {
            if (circle.contains(ordered[j])) continue;
            circle = circle.r <= EPS ? diameter(ordered[i], ordered[j]) :
                twoBoundary(ordered, j + 1, ordered[i], ordered[j]);
        }
    }
    for (Point p : points) circle.r = std::max(circle.r, dist(circle.c, p));
    return circle;
}
pair<Point,Point> diameterPair(const Poly& p){pair<Point,Point> z{p[0],p[0]};double best=0;for(size_t i=0;i<p.size();++i)for(size_t j=i;j<p.size();++j)if(dist(p[i],p[j])>best){best=dist(p[i],p[j]);z={p[i],p[j]};}return z;}
// The smallest six-point ring that still covers the 1800 m disk with a
// 1000 m reception disk has radius about 1122.96 m.  Keep a small numerical
// margin while avoiding the older 1150 m over-expansion.
vector<Point> searchPoints(){vector<Point> p{{0,0}};for(int k=0;k<6;++k)p.push_back(unit(k*60)*1125.0);return p;}
vector<int> route(Point start,const std::map<int,Point>& d){
    std::map<int,bool> used;vector<int> r;Point q=start;while(r.size()<d.size()){int k=-1;double b=1e100;for(auto& x:d)if(!used[x.first]&&(k<0||dist(q,x.second)<b)){k=x.first;b=dist(q,x.second);}used[k]=true;r.push_back(k);q=d.at(k);}
    bool change=true;auto at=[&](int k){return d.at(k);};while(change){change=false;for(size_t i=0;i+1<r.size();++i)for(size_t j=i+1;j<r.size();++j){Point prev=i?at(r[i-1]):start;Point first=at(r[i]),last=at(r[j]);double old=dist(prev,first),nw=dist(prev,last);if(j+1<r.size()){old+=dist(last,at(r[j+1]));nw+=dist(first,at(r[j+1]));}if(nw+1e-7<old){std::reverse(r.begin()+i,r.begin()+j+1);change=true;}}}return r;
}

using Json = nlohmann::json;

bool has(const string& s, const string& key, const string& value) {
    Json j = Json::parse(s);
    return j.contains(key) && j.at(key).is_string() && j.at(key).get<string>() == value;
}

double number(const string& s, const string& key) {
    Json j = Json::parse(s);
    if (!j.contains(key) || !j.at(key).is_number())
        throw std::runtime_error("Missing or invalid numeric field: " + key);
    double result = j.at(key).get<double>();
    if (!std::isfinite(result)) throw std::runtime_error("Nonfinite field: " + key);
    return result;
}

double nowSeconds() { return GetTickCount64() / 1000.0; }
struct BudgetExceeded : std::runtime_error { using std::runtime_error::runtime_error; };
struct HttpStatusError : std::runtime_error { using std::runtime_error::runtime_error; };

struct ITransport {
    virtual string post(const string&, const string&, double) = 0;
    virtual ~ITransport() = default;
};

struct HttpHandle {
    HINTERNET value;
    explicit HttpHandle(HINTERNET v) : value(v) {}
    ~HttpHandle() { if (value) WinHttpCloseHandle(value); }
    HttpHandle(const HttpHandle&) = delete;
    HttpHandle& operator=(const HttpHandle&) = delete;
};

struct WinHttpTransport : ITransport {
    string host;
    INTERNET_PORT port = 2026;

    explicit WinHttpTransport(string url) {
        const string prefix = "http://";
        if (url.compare(0, prefix.size(), prefix) != 0)
            throw std::runtime_error("URL must start with http://");
        string origin = url.substr(prefix.size());
        if (!origin.empty() && origin.back() == '/') origin.pop_back();
        size_t colon = origin.find(':');
        host = origin.substr(0, colon);
        if (colon != string::npos) {
            size_t used = 0;
            string text = origin.substr(colon + 1);
            int p = std::stoi(text, &used);
            if (used != text.size() || p < 1 || p > 65535)
                throw std::runtime_error("Invalid port");
            port = static_cast<INTERNET_PORT>(p);
        }
        if (host != "127.0.0.1" && host != "localhost")
            throw std::runtime_error("Use the local simulator URL (127.0.0.1 or localhost)");
    }

    string once(const string& path, const string& body, double deadline) {
        HttpHandle session(WinHttpOpen(L"CUMCM2026B-problem3/1.0",
                                      WINHTTP_ACCESS_TYPE_NO_PROXY, nullptr, nullptr, 0));
        if (!session.value) throw std::runtime_error("WinHttpOpen failed");
        std::wstring wh(host.begin(), host.end()), wp(path.begin(), path.end());
        HttpHandle connection(WinHttpConnect(session.value, wh.c_str(), port, 0));
        if (!connection.value) throw std::runtime_error("WinHttpConnect failed");
        HttpHandle action(WinHttpOpenRequest(connection.value, L"POST", wp.c_str(), nullptr,
                                             WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, 0));
        if (!action.value) throw std::runtime_error("WinHttpOpenRequest failed");
        int timeout = static_cast<int>(std::max(1.0, std::min(5000.0, (deadline - nowSeconds()) * 1000)));
        WinHttpSetTimeouts(action.value, timeout, timeout, timeout, timeout);
        DWORD redirect = WINHTTP_OPTION_REDIRECT_POLICY_NEVER;
        WinHttpSetOption(action.value, WINHTTP_OPTION_REDIRECT_POLICY, &redirect, sizeof(redirect));
        const wchar_t* header = L"Content-Type: application/json";
        if (!WinHttpSendRequest(action.value, header, static_cast<DWORD>(-1),
                               const_cast<char*>(body.data()), static_cast<DWORD>(body.size()),
                               static_cast<DWORD>(body.size()), 0) ||
            !WinHttpReceiveResponse(action.value, nullptr))
            throw std::runtime_error("HTTP connection failed (WinHTTP " + std::to_string(GetLastError()) + ")");
        DWORD status = 0, size = sizeof(status);
        if (!WinHttpQueryHeaders(action.value, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                                 nullptr, &status, &size, nullptr))
            throw std::runtime_error("Cannot read HTTP status");
        if (status != 200) throw HttpStatusError("HTTP " + std::to_string(status) + " on " + path);
        string output;
        while (true) {
            if (nowSeconds() >= deadline) throw BudgetExceeded("Real-time deadline during response");
            DWORD available = 0;
            if (!WinHttpQueryDataAvailable(action.value, &available))
                throw std::runtime_error("Incomplete HTTP response");
            if (!available) break;
            if (output.size() + available > 1048576) throw HttpStatusError("Response too large");
            vector<char> buffer(available);
            DWORD received = 0;
            if (!WinHttpReadData(action.value, buffer.data(), available, &received) || !received)
                throw std::runtime_error("HTTP response was interrupted");
            output.append(buffer.data(), received);
        }
        return output;
    }

    string post(const string& path, const string& body, double deadline) override {
        // The caller serializes one action once. All retries use that exact body/ID.
        for (int attempt = 0; attempt < 4; ++attempt) {
            if (nowSeconds() >= deadline) throw BudgetExceeded("Real-time deadline before HTTP request");
            try { return once(path, body, deadline); }
            catch (const HttpStatusError&) { throw; }
            catch (const BudgetExceeded&) { throw; }
            catch (const std::exception&) {
                if (attempt == 3) throw;
                Sleep(static_cast<DWORD>(std::max(0.0, std::min(250.0 * (1 << attempt),
                                                              (deadline - nowSeconds()) * 1000))));
            }
        }
        throw std::runtime_error("HTTP retry limit");
    }
};

struct Session {
    struct PhaseStats {
        double virtualTime = 0, distance = 0;
        int measures = 0, switches = 0, clears = 0, failures = 0;
    };
    ITransport& io;
    string robot, id;
    int seq = 0, current = 1;
    Point pos;
    double vt = 0, deadline = 1e100, maxVirtual = 360000, start = 0, finish = 0;
    bool entered = false, uncertain = false;
    std::ofstream log;
    double distanceMoved = 0;
    int measures = 0, switches = 0, cleared = 0, failed = 0;
    string phase = "search";
    std::map<string, PhaseStats> phases;

    Session(ITransport& transport, string robotId, const string& logPath)
        : io(transport), robot(std::move(robotId)) {
        if (robot.empty() || robot.size() > 64 ||
            std::any_of(robot.begin(), robot.end(), [](unsigned char c) { return c < 32 || c == 127; }))
            throw std::runtime_error("Invalid robot_id");
        id = "p3-" + std::to_string(GetTickCount64()) + "-" + std::to_string(GetCurrentProcessId()) + "-";
        if (!logPath.empty()) {
            size_t slash = logPath.find_last_of("/\\");
            if (slash != string::npos) {
                string parent = logPath.substr(0, slash);
                if (!CreateDirectoryA(parent.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS)
                    throw std::runtime_error("Cannot create log directory: " + parent);
            }
            if (std::ifstream(logPath).good()) throw std::runtime_error("Log already exists; use a new --log path");
            log.open(logPath);
            if (!log) throw std::runtime_error("Cannot open log: " + logPath);
        }
    }

    void record(const Json& event) {
        if (log.is_open()) { log << event.dump() << '\n'; log.flush(); }
    }

    string call(const string& path, Json extra = Json::object()) {
        if (uncertain) throw std::runtime_error("Previous action outcome is unknown; do not send a new action");
        extra["arena_id"] = "default";
        extra["robot_id"] = robot;
        extra["request_id"] = id + std::to_string(++seq);
        string body = extra.dump();
        record({{"kind", "request"}, {"path", path}, {"phase", phase}, {"payload", extra}});
        string output;
        Json response;
        try {
            output = io.post(path, body, deadline);
            response = Json::parse(output);
        } catch (...) {
            uncertain = true;
            record({{"kind", "uncertain_action"}, {"path", path}, {"request_id", extra["request_id"]}});
            throw;
        }
        record({{"kind", "response"}, {"path", path}, {"response", response}});
        if (!response.is_object() || !response.contains("accepted") ||
            !response["accepted"].is_boolean() || !response["accepted"].get<bool>())
            throw std::runtime_error(path + " rejected; check robot_id and simulator state");
        double nextTime = number(output, "virtual_time_s");
        if (nextTime + 1e-5 < vt) throw std::runtime_error("Nonmonotonic virtual clock");
        vt = nextTime;
        return output;
    }

    void enter() {
        if (entered) throw std::runtime_error("Already entered");
        start = nowSeconds();
        string output = call("/enter");
        entered = true;
        deadline = start + number(output, "remaining_real_duration_s");
        maxVirtual = number(output, "max_virtual_duration_s");
    }

    string action(const string& path, Point point, int channel) {
        if (!entered) throw std::runtime_error("Not entered");
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || std::abs(point.x) > 2000000 ||
            std::abs(point.y) > 2000000 || channel < 1 || channel > 20)
            throw std::runtime_error("Invalid action coordinates/channel");
        double movement = dist(pos, point);
        int switched = path == "/measure" && channel != current;
        if (nowSeconds() >= deadline - 3 || vt + movement / 5 + 5 + switched >= maxVirtual - 1)
            throw BudgetExceeded("Time reserved for /exit");
        double previousTime = vt;
        string output = call(path, {{"position", {{"x", point.x}, {"y", point.y}}}, {"channel", channel}});
        pos = point;
        distanceMoved += movement;
        PhaseStats& stats = phases[phase];
        stats.virtualTime += vt - previousTime;
        stats.distance += movement;
        if (path == "/measure") {
            ++measures; switches += switched; current = channel;
            ++stats.measures; stats.switches += switched;
            if (has(output, "measure_result", "direction")) {
                double angle = number(output, "svd_deg");
                if (angle < 0 || angle >= 360) throw std::runtime_error("Invalid bearing");
            } else if (!has(output, "measure_result", "near") && !has(output, "measure_result", "no_signal"))
                throw std::runtime_error("Unknown measure_result");
        } else {
            if (has(output, "clear_result", "success")) { ++cleared; ++stats.clears; }
            else if (has(output, "clear_result", "no_target_in_range")) { ++failed; ++stats.failures; }
            else throw std::runtime_error("Unknown clear_result");
        }
        return output;
    }
    string measure(Point p, int channel) { return action("/measure", p, channel); }
    string clear(Point p, int channel) { return action("/clear", p, channel); }
    void exit() {
        if (entered && !uncertain) { call("/exit"); entered = false; finish = nowSeconds(); }
    }
    Json summary() const {
        double modelTime = distanceMoved / 5 + 5 * measures + switches + 5 * cleared + 3 * failed;
        Json result = {{"cleared", cleared}, {"measurements", measures}, {"switches", switches},
                       {"failed_clears", failed}, {"distance_m", distanceMoved}, {"virtual_time_s", vt},
                       {"average_time_s", cleared ? Json(vt / cleared) : Json(nullptr)},
                       {"program_time_s", start ? (finish ? finish : nowSeconds()) - start : 0},
                       {"time_accounting_error_s", vt - modelTime}};
        result["phases"] = Json::object();
        for (const auto& entry : phases) {
            const auto& stats = entry.second;
            result["phases"][entry.first] = {
                {"virtual_time_s", stats.virtualTime}, {"distance_m", stats.distance},
                {"measurements", stats.measures}, {"switches", stats.switches},
                {"cleared", stats.clears}, {"failed_clears", stats.failures}};
        }
        return result;
    }
};

Json completionMetrics(int cleared, int total, double elapsed) {
    return {{"cleared", cleared}, {"total_sources", total >= 0 ? Json(total) : Json(nullptr)},
        {"localization_clear_time_s", elapsed},
        {"cleared_ratio", total > 0 ? Json(static_cast<double>(cleared) / total) : Json(nullptr)},
        {"average_time_s", cleared > 0 ? Json(elapsed / cleared) : Json(nullptr)}};
}

void printCompletionMetrics(const Json& metrics, bool batch = false) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(2);
    output << (batch ? u8"\n离线批量测试汇总（全部案例合计）\n" : u8"\n任务结果汇总\n");
    output << u8"被清除干扰源的个数: " << metrics.at("cleared").get<int>() << '\n';
    output << u8"干扰源总数: ";
    if (metrics.at("total_sources").is_null()) output << u8"未知（尚未完成搜索）";
    else output << metrics.at("total_sources").get<int>();
    output << '\n' << u8"定位清除总时间: " << metrics.at("localization_clear_time_s").get<double>() << u8" 秒\n";
    output << u8"被清除干扰源个数的比例: ";
    if (metrics.at("cleared_ratio").is_null()) output << u8"无法计算";
    else output << 100 * metrics.at("cleared_ratio").get<double>() << '%';
    output << '\n' << u8"平均定位清除时间: ";
    if (metrics.at("average_time_s").is_null()) output << u8"无法计算（尚未清除干扰源）";
    else output << metrics.at("average_time_s").get<double>() << u8" 秒/个";
    std::cout << output.str() << std::endl;
}

struct Target {
    int channel = 0;
    vector<pair<Point, double>> obs;
    Poly region;
    vector<Point> secondRegion;
    vector<Point> failedClearPoints;
    bool probeAttempted = false;
    Circle c;
    string status = "detected";
    double discoveredTime = 0, locatedTime = -1, clearedTime = -1;
};

bool inDisks(Point point, const vector<Point>& centers, double radius) {
    for (Point center : centers) if (dist(point, center) > radius + 1e-7) return false;
    return true;
}

bool projectToDisks(Point point, const vector<Point>& centers, double radius, Point& result) {
    bool found = false;
    double best = 1e100;
    auto accept = [&](Point candidate) {
        if (!inDisks(candidate, centers, radius)) return;
        double distance = dist(point, candidate);
        if (distance < best) { best = distance; result = candidate; found = true; }
    };
    accept(point);
    if (found) return true;
    // A projection onto a disk intersection lies on one circle or at a
    // circle intersection. Enumerating these supports is exact in 2D.
    for (size_t i = 0; i < centers.size(); ++i) {
        Point offset = point - centers[i];
        if (norm(offset) > EPS) accept(centers[i] + offset * (radius / norm(offset)));
        for (size_t j = 0; j < i; ++j) {
            Point axis = centers[i] - centers[j];
            double length = norm(axis);
            if (length <= EPS || length > 2 * radius) continue;
            Point middle = (centers[i] + centers[j]) * .5;
            Point normal{-axis.y / length, axis.x / length};
            double height = std::sqrt(std::max(0.0, radius * radius - length * length / 4));
            accept(middle + normal * height);
            accept(middle - normal * height);
        }
    }
    return found;
}

#include "one_read_region.hpp"
#include "clear_region.hpp"
#include "probe_clear.hpp"
#include "bearing_estimate.hpp"

struct Planner {
    Session& s;
    std::map<int, Target> targets;
    std::set<int> done, absent;
    int scanned = 0, homingMoves = 0;
    bool quiet = false, complete = false;
    int sharedMeasures = 0, sharedLocations = 0, sharedMisses = 0;
    int centerMeasures = 0;
    int oneReadRegions = 0, oneReadCandidates = 0, oneReadMeasures = 0;
    int oneReadOffCenter = 0, oneReadFailures = 0;
    int probeAttempts = 0, probeSuccesses = 0, probeFailures = 0;
    int areaCreated = 0, areaRetired = 0, areaMeasures = 0, areaMisses = 0;
    int unifiedDispatches = 0, unifiedSearchChoices = 0, unifiedLocalChoices = 0;
    int unifiedMixedWindows = 0, unifiedRefinements = 0, unifiedClearsDuringSearch = 0;
    int unifiedMaxWindow = 0, unifiedGuaranteedMisses = 0, actionsSinceSearch = 0;
    int unifiedS2Windows = 0, unifiedRegionWindows = 0, unifiedMultiBeforeSearch = 0;
    vector<int> stationVisitOrder;
    vector<RouteGroup> lastWindow;
    RouteDecision lastDecision;
    int contourCandidates = 0;
    std::map<int, int> attempts;
    std::map<int, vector<Point>> measuredPoints, negativePoints;
    vector<Point> search = searchPoints();

    explicit Planner(Session& session, bool silent = false) : s(session), quiet(silent) {}

    bool measuredAt(int channel, Point point) const {
        auto history = measuredPoints.find(channel);
        if (history == measuredPoints.end()) return false;
        for (Point old : history->second) if (dist(old, point) <= 1e-5) return true;
        return false;
    }

    void markLocated(Target& target) {
        retireArea(target);
        target.status = "located";
        if (target.locatedTime < 0) target.locatedTime = s.vt;
    }

    void retireArea(Target& target) {
        if (!target.secondRegion.empty()) ++areaRetired;
        target.secondRegion.clear();
    }

    Point clearPoint(const Target& target, Point from) const {
        Point point;
        if (!clear_region::nearest(target.region, target.c, from, point))
            throw std::runtime_error("No certified clearing position");
        return point;
    }

    void finishClear(Target& target) {
        done.insert(target.channel);
        retireArea(target);
        target.status = "cleared";
        target.clearedTime = s.vt;
        if (target.locatedTime < 0) target.locatedTime = s.vt - 5;
        if (!quiet) std::cout << "cleared channel " << target.channel << "; total=" << done.size() << '\n';
    }

    void clearTarget(Target& target, Point point) {
        string previousPhase = s.phase;
        s.phase = "clear";
        if (!has(s.clear(point, target.channel), "clear_result", "success"))
            throw std::runtime_error("Certified clear failed for channel " + std::to_string(target.channel));
        s.phase = previousPhase;
        finishClear(target);
    }

    bool tryProbeClear(Target& target, Point point) {
        if (target.status != "detected" || target.obs.size() < 2 || target.probeAttempted ||
            target.c.r > 80 || probe_clear::coverage(target.region, point) < .4) return false;
        target.probeAttempted = true;
        ++probeAttempts;
        s.record({{"kind", "probe_clear_plan"}, {"channel", target.channel},
            {"point", {point.x, point.y}}, {"radius_m", target.c.r},
            {"area_coverage", probe_clear::coverage(target.region, point)}});
        string previousPhase = s.phase;
        s.phase = "probe_clear";
        string output = s.clear(point, target.channel);
        s.phase = previousPhase;
        if (has(output, "clear_result", "success")) {
            ++probeSuccesses;
            finishClear(target);
            return true;
        }
        ++probeFailures;
        target.failedClearPoints.push_back(point);
        // A failed 20 m clear is not a 1000 m no-signal observation.
        // Keep the real polygon and all measurement histories unchanged.
        return false;
    }

    Target* observe(Point point, int channel) {
        bool areaReading = false;
        auto oldTarget = targets.find(channel);
        bool certifiedReading = oldTarget != targets.end() && oldTarget->second.status == "detected" &&
            oldTarget->second.obs.size() >= 2 && one_read_region::contains(oldTarget->second.region, point);
        if (certifiedReading) {
            ++oneReadMeasures;
            if (dist(point, oldTarget->second.c.c) > 1e-5) ++oneReadOffCenter;
        }
        if (oldTarget != targets.end())
            for (Point candidate : oldTarget->second.secondRegion)
                if (dist(candidate, point) < 1e-5) { areaReading = true; break; }
        if (areaReading) ++areaMeasures;
        string output = s.measure(point, channel);
        measuredPoints[channel].push_back(point);
        if (has(output, "measure_result", "no_signal")) {
            if (certifiedReading) ++oneReadFailures;
            if (areaReading) ++areaMisses;
            negativePoints[channel].push_back(point);
            return nullptr;
        }
        bool first = targets.count(channel) == 0;
        Target& target = targets[channel];
        target.channel = channel;
        if (first) target.discoveredTime = s.vt;
        if (has(output, "measure_result", "near")) {
            clearTarget(target, point);
            return &target;
        }
        double angle = number(output, "svd_deg");
        Poly previous = target.region.empty() ? outerDisk({0, 0}, 1800) : target.region;
        Poly next = disk(wedge(previous, point, angle), point, 1500);
        if (next.empty()) throw std::runtime_error("Inconsistent observations for channel " + std::to_string(channel));
        target.region = std::move(next);
        target.obs.push_back({point, angle});
        target.c = mec(target.region);
        if (target.obs.size() == 1) {
            for (Point local : area_region::samples()) {
                Point candidate = area_region::toWorld(local, point, angle);
                if (guaranteedReception(target, candidate)) target.secondRegion.push_back(candidate);
            }
            if (!target.secondRegion.empty()) ++areaCreated;
        } else retireArea(target);
        if (target.c.r <= 19.9) markLocated(target);
        else if (certifiedReading) ++oneReadFailures;
        return &target;
    }

    double farthest(const Target& target, Point point) const {
        double maximum = 0;
        for (Point p : target.region) maximum = std::max(maximum, dist(p, point));
        return maximum;
    }

    vector<Point> receptionLens(const pair<Point, double>& observation) const {
        return {observation.first, observation.first + unit(observation.second - ERR) * 1000,
                observation.first + unit(observation.second + ERR) * 1000};
    }

    bool guaranteedReception(const Target& target, Point point) const {
        if (farthest(target, point) <= 999.8) return true;
        // A successful reading at A implies r_i >= max(1000, |A-source|).
        // Each three-disk lens ensures the next distance is below that bound
        // for every direction in the measured wedge, including ranges >1000.
        for (const auto& observation : target.obs)
            if (inDisks(point, receptionLens(observation), 999.8)) return true;
        return false;
    }

    double receptionLowerBound(const Target& target, Point source) const {
        double radius = 1000;
        for (const auto& observation : target.obs)
            radius = std::max(radius, dist(source, observation.first));
        return radius;
    }

    vector<Point> areaCandidates(const Target& target, const vector<Point>& anchors,
                                 size_t limit = 10) const {
        vector<Point> candidates;
        if (target.secondRegion.empty()) return candidates;
        auto add = [&](Point point) {
            if (measuredAt(target.channel, point)) return;
            for (Point old : candidates) if (dist(old, point) < 20) return;
            candidates.push_back(point);
        };
        Point firstDirection = unit(target.obs.front().second);
        for (int side : {-1, 1}) {
            double best = 1e100;
            Point selected;
            for (Point point : target.secondRegion) {
                if (side * cross(firstDirection, point - target.obs.front().first) <= 0) continue;
                double cost = anchors.empty() ? 0 : dist(point, anchors.front());
                if (cost < best) { best = cost; selected = point; }
            }
            if (best < 1e100) add(selected);
        }
        // Nearest points and shortest insertions respond to both route neighbors.
        for (Point anchor : anchors) {
            auto nearest = std::min_element(target.secondRegion.begin(), target.secondRegion.end(),
                [&](Point a, Point b) { return dist(a, anchor) < dist(b, anchor); });
            add(*nearest);
        }
        for (size_t i = 0; i + 1 < anchors.size(); ++i) {
            auto nearest = std::min_element(target.secondRegion.begin(), target.secondRegion.end(),
                [&](Point a, Point b) {
                    return dist(a, anchors[i]) + dist(a, anchors[i + 1]) <
                           dist(b, anchors[i]) + dist(b, anchors[i + 1]);
                });
            add(*nearest);
        }
        // Retain both disconnected branches and spatially separated alternatives.
        while (candidates.size() < limit) {
            double best = -1;
            Point point;
            for (Point candidate : target.secondRegion) {
                if (measuredAt(target.channel, candidate)) continue;
                double nearest = 1e100;
                for (Point old : candidates) nearest = std::min(nearest, dist(old, candidate));
                if (nearest > best) { best = nearest; point = candidate; }
            }
            if (best < 20) break;
            add(point);
        }
        if (candidates.size() > limit) candidates.resize(limit);
        return candidates;
    }

    vector<Point> sampleSources(const Target& target) const {
        auto endpoints = diameterPair(target.region);
        vector<Point> samples{target.c.c};
        for (int i = 0; i <= 8; ++i)
            samples.push_back(endpoints.first + (endpoints.second - endpoints.first) * (i / 8.0));
        for (size_t i = 0; i < std::min<size_t>(4, target.region.size()); ++i)
            samples.push_back(target.region[i * target.region.size() / std::min<size_t>(4, target.region.size())]);
        auto history = negativePoints.find(target.channel);
        if (history != negativePoints.end()) {
            samples.erase(std::remove_if(samples.begin(), samples.end(), [&](Point point) {
                for (Point station : history->second) if (dist(point, station) < 1000 - 1e-6) return true;
                return false;
            }), samples.end());
        }
        return samples;
    }

    static double refinementCost(double radius) {
        if (radius <= 19.9) return 0;
        // A time-valued heuristic, not a bound or an assumed source distribution.
        return 2 * (radius - 19.9) / 5 + 5 * std::ceil(std::log2(radius / 19.9));
    }

    struct Prediction {
        double radius = 0, remainingTime = 0, receptionFraction = 0;
        bool valid = false;
    };
    struct CandidatePrediction { Point point; Prediction prediction; };
    struct CandidateCache { size_t observations = 0, misses = 0; vector<CandidatePrediction> options; };
    std::map<int, CandidateCache> candidateCache;

    double remainingCost(Circle circle, Point from) const {
        if (circle.r <= 19.9)
            return std::max(0.0, dist(from, circle.c) - (19.9 - circle.r)) / 5;
        // Repeated center readings halve the radius; their future positions are unknown.
        const double q = 1 / (2 * std::cos(ERR * PI / 180));
        int readings = static_cast<int>(std::ceil(std::log(19.9 / circle.r) / std::log(q)));
        double futureTravel = q * circle.r * (1 - std::pow(q, readings)) / (1 - q);
        return dist(from, circle.c) / 5 + 5 * readings + futureTravel / 5;
    }

    Prediction predict(const Target& target, Point candidate, bool useReceptionHistory = false) const {
        Prediction result;
        vector<Point> samples = sampleSources(target);
        if (samples.empty()) return result;
        for (Point source : samples) {
            double distance = dist(candidate, source);
            // Beyond 1000 m, conservatively rank this sample as a missed reading.
            // This does not remove any point from the certified convex region.
            if (distance > (useReceptionHistory ? receptionLowerBound(target, source) : 1000)) {
                result.radius += target.c.r;
                result.remainingTime += distance / 5 + refinementCost(target.c.r);
                continue;
            }
            result.receptionFraction += 1;
            if (distance <= 5) { result.radius += 5; continue; }
            double worstRadius = 0, worstRemaining = 0;
            for (double error : {-ERR, 0.0, ERR}) {
                Poly hypothetical = wedge(target.region, candidate, bearing(candidate, source) + error);
                if (hypothetical.empty()) continue;
                Circle circle = mec(hypothetical);
                worstRadius = std::max(worstRadius, circle.r);
                worstRemaining = std::max(worstRemaining, remainingCost(circle, candidate));
            }
            result.radius += worstRadius;
            result.remainingTime += worstRemaining;
        }
        result.radius /= samples.size();
        result.remainingTime /= samples.size();
        result.receptionFraction /= samples.size();
        result.valid = true;
        return result;
    }

    double onwardCost(const Target& target, const Poly& region, Point source, double error,
                      const vector<Point>& future) const {
        Circle circle = mec(region);
        double best = remainingCost(circle, future.back());
        // Both alternatives end at the same station. Account for a useful
        // later shared reading before assigning value to an earlier stop.
        for (Point station : future) {
            if (measuredAt(target.channel, station) ||
                dist(source, station) > receptionLowerBound(target, source)) continue;
            if (dist(source, station) <= 5) { best = std::min(best, 6.0); continue; }
            Poly after = wedge(region, station, bearing(station, source) + error);
            if (!after.empty()) best = std::min(best, 6 + remainingCost(mec(after), future.back()));
        }
        return best;
    }

    double detourGain(const Target& target, Point candidate, const vector<Point>& future,
                      double actionCost) const {
        vector<Point> samples = sampleSources(target);
        if (samples.empty()) return -1e100;
        double gain = 0;
        for (Point source : samples) {
            double before = 0, after = 0;
            for (double error : {-ERR, 0.0, ERR}) {
                before = std::max(before, onwardCost(target, target.region, source, error, future));
                if (dist(candidate, source) <= 5) continue;
                Poly updated = wedge(target.region, candidate, bearing(candidate, source) + error);
                if (updated.empty()) return -1e100;
                after = std::max(after, onwardCost(target, updated, source, error, future));
            }
            gain += before - after;
        }
        return gain / samples.size() - actionCost;
    }


    void appendContourCandidates(const Target& target, vector<Point>& candidates) {
        // A continuous contour cannot be executed or enumerated.  These four
        // radial levels are a bounded geometric approximation around the MEC;
        // the actual score still includes reception, travel and future work.
        const double radius = target.c.r;
        if (radius < 40.0) return;
        const vector<double> levels{0.25, 0.50, 0.75, 1.00};
        vector<Point> contour;
        const int angularSamples = radius > 160.0 ? 16 : 12;
        for (double level : levels) {
            double ring = radius * level;
            for (int k = 0; k < angularSamples; ++k) {
                double angle = 360.0 * k / angularSamples;
                contour.push_back(target.c.c + unit(angle) * ring);
            }

            // Add exact intersections of the ring with the current polygon.
            // This preserves useful narrow-region directions that a coarse
            // angular grid can miss.
            for (size_t i = 0; i < target.region.size(); ++i) {
                Point a = target.region[i];
                Point d = target.region[(i + 1) % target.region.size()] - a;
                Point q = a - target.c.c;
                double aa = dot(d, d);
                if (aa <= EPS) continue;
                double bb = 2 * dot(q, d);
                double cc = dot(q, q) - ring * ring;
                double discriminant = bb * bb - 4 * aa * cc;
                if (discriminant < -1e-7) continue;
                double root = std::sqrt(std::max(0.0, discriminant));
                for (double t : {(-bb - root) / (2 * aa), (-bb + root) / (2 * aa)}) {
                    if (t >= -1e-7 && t <= 1.0 + 1e-7)
                        contour.push_back(a + d * std::max(0.0, std::min(1.0, t)));
                }
            }
        }

        // Polygon clipping can produce many nearly duplicate edge hits. Remove
        // them before applying the deterministic budget so prediction work is
        // spent on distinct geometry rather than repeated vertices.
        vector<Point> unique;
        unique.reserve(contour.size());
        for (Point point : contour) {
            bool duplicate = false;
            for (Point old : candidates)
                if (dist(old, point) <= 1e-5) { duplicate = true; break; }
            if (!duplicate) for (Point old : unique)
                if (dist(old, point) <= 1e-5) { duplicate = true; break; }
            if (!duplicate) unique.push_back(point);
        }
        contour.swap(unique);

        // Keep a deterministic budget so prediction remains cheap in large
        // tests.
        const size_t contourBudget = 96;
        if (contour.size() > contourBudget) {
            vector<Point> reduced;
            reduced.reserve(contourBudget);
            for (size_t i = 0; i < contourBudget; ++i) {
                size_t index = std::min(contour.size() - 1,
                    static_cast<size_t>(i * contour.size() / contourBudget));
                reduced.push_back(contour[index]);
            }
            contour.swap(reduced);
        }
        contourCandidates += static_cast<int>(contour.size());
        candidates.insert(candidates.end(), contour.begin(), contour.end());
    }

    vector<Point> geometryCandidates(const Target& target) {
        Point center = target.c.c;
        auto endpoints = diameterPair(target.region);
        Point axis = endpoints.second - endpoints.first;
        double length = std::max(EPS, norm(axis));
        Point normal{-axis.y / length, axis.x / length};
        auto first = target.obs.front(), last = target.obs.back();
        Point e = unit(first.second), n{-e.y, e.x};
        double upper = std::min(1500.0, farthest(target, last.first));
        Point contraction = last.first + unit(last.second) * (upper / (2 * std::cos(ERR * PI / 180)));
        vector<Point> candidates{s.pos, center, contraction,
                                 first.first + e * 667 + n * 745, first.first + e * 667 - n * 745};
        if (target.obs.size() >= 2) candidates.push_back(bearing_estimate::estimate(target));
        for (double fraction : {.25, .5, .75})
            candidates.push_back(endpoints.first + axis * fraction);
        Point nearest = center;
        for (size_t i = 0; i < target.region.size(); ++i) {
            Point a = target.region[i], edge = target.region[(i + 1) % target.region.size()] - a;
            double fraction = std::max(0.0, std::min(1.0, dot(s.pos - a, edge) / std::max(EPS, dot(edge, edge))));
            Point point = a + edge * fraction;
            if (dist(s.pos, point) < dist(s.pos, nearest)) nearest = point;
        }
        candidates.push_back(nearest);
        candidates.push_back((nearest + center) * .5);
        for (double offset : {75.0, 150.0, 300.0, 500.0, 750.0}) {
            candidates.push_back(center + normal * offset);
            candidates.push_back(center - normal * offset);
        }
        appendContourCandidates(target, candidates);
        return candidates;
    }


    void executeUnifiedTarget(Target& target, Point point) {
        if (target.status == "located") {
            clearTarget(target, point);
            return;
        }
        if (farthest(target, s.pos) <= 19.9) {
            clearTarget(target, s.pos);
            return;
        }
        if (tryProbeClear(target, point)) return;
        bool guaranteed = guaranteedReception(target, point);
        if (dist(point, target.c.c) < 1e-5) ++centerMeasures;
        double previous = target.c.r;
        bool received = observe(point, target.channel) != nullptr;
        if (guaranteed && !received) ++unifiedGuaranteedMisses;
        ++attempts[target.channel];
        if (target.status == "detected" &&
            (!received || target.c.r >= .98 * previous || attempts[target.channel] >= 6))
            homing(target);
        if (target.status == "located" && farthest(target, s.pos) <= 19.9)
            clearTarget(target, s.pos);
    }

    vector<CandidatePrediction> taskCandidates(const Target& target, const vector<Point>& anchors) {
        CandidateCache& cache = candidateCache[target.channel];
        if (cache.observations != target.obs.size() || cache.misses != negativePoints[target.channel].size()) {
            cache.observations = target.obs.size();
            cache.misses = negativePoints[target.channel].size();
            cache.options.clear();
            vector<Point> points = geometryCandidates(target);
            vector<Point> area = areaCandidates(target, anchors, 10);
            points.insert(points.end(), area.begin(), area.end());
            for (Point point : points) {
                if (measuredAt(target.channel, point) || !guaranteedReception(target, point)) continue;
                bool duplicate = false;
                for (const auto& old : cache.options) if (dist(old.point, point) < 1e-5) duplicate = true;
                if (duplicate) continue;
                Prediction prediction = predict(target, point, true);
                if (prediction.valid) cache.options.push_back({point, prediction});
            }
        }
        vector<CandidatePrediction> pool = cache.options;
        vector<Point> dynamic = areaCandidates(target, anchors, 8);
        vector<Point> certified;
        if (target.obs.size() >= 2) {
            certified = one_read_region::candidates(target.region, target.c, anchors);
            if (!certified.empty()) ++oneReadRegions;
            oneReadCandidates += static_cast<int>(certified.size());
            dynamic.insert(dynamic.end(), certified.begin(), certified.end());
        }
        dynamic.push_back(s.pos);
        for (Point anchor : anchors) {
            Point nearest = target.c.c;
            for (size_t i = 0; i < target.region.size(); ++i) {
                Point a = target.region[i], edge = target.region[(i + 1) % target.region.size()] - a;
                double t = std::max(0.0, std::min(1.0, dot(anchor - a, edge) / std::max(EPS, dot(edge, edge))));
                Point point = a + edge * t;
                if (dist(anchor, point) < dist(anchor, nearest)) nearest = point;
            }
            dynamic.push_back(nearest);
            dynamic.push_back((nearest + target.c.c) * .5);
        }
        for (Point point : dynamic) {
            if (measuredAt(target.channel, point) || !guaranteedReception(target, point)) continue;
            bool duplicate = false;
            for (const auto& old : pool) if (dist(old.point, point) < 1e-5) duplicate = true;
            if (!duplicate) {
                Prediction prediction = predict(target, point, true);
                if (prediction.valid) pool.push_back({point, prediction});
            }
        }
        vector<CandidatePrediction> selected;
        auto add = [&](const CandidatePrediction& candidate) {
            if (measuredAt(target.channel, candidate.point)) return;
            for (const auto& old : selected) if (dist(old.point, candidate.point) < 1) return;
            if (selected.size() < 10) selected.push_back(candidate);
        };
        auto retain = [&](Point point) {
            auto candidate = std::find_if(pool.begin(), pool.end(), [&](const CandidatePrediction& option) {
                return dist(point, option.point) < 1e-5;
            });
            if (candidate != pool.end()) add(*candidate);
        };
        // A real two-bearing region always keeps its center as a baseline.
        // Certified one-reading alternatives keep priority in the finite window.
        if (target.obs.size() >= 2) {
            retain(target.c.c);
            retain(bearing_estimate::estimate(target));
        }
        for (Point point : certified) retain(point);
        // Keep each disconnected S2 branch represented in the joint solve.
        for (Point point : areaCandidates(target, anchors, 2)) {
            auto option = std::find_if(pool.begin(), pool.end(), [&](const CandidatePrediction& candidate) {
                return dist(point, candidate.point) < 1e-5;
            });
            if (option != pool.end()) add(*option);
        }
        // Retain alternatives selected from different predecessor positions;
        // the route solver, not this shortlist, makes the final location choice.
        for (Point anchor : anchors) {
            auto best = std::min_element(pool.begin(), pool.end(), [&](const CandidatePrediction& a,
                                                                      const CandidatePrediction& b) {
                return dist(anchor, a.point) / 5 + a.prediction.remainingTime <
                       dist(anchor, b.point) / 5 + b.prediction.remainingTime;
            });
            if (best != pool.end()) add(*best);
        }
        std::sort(pool.begin(), pool.end(), [&](const CandidatePrediction& a, const CandidatePrediction& b) {
            return dist(s.pos, a.point) / 5 + a.prediction.remainingTime <
                   dist(s.pos, b.point) / 5 + b.prediction.remainingTime;
        });
        for (const auto& candidate : pool) {
            bool close = false;
            for (const auto& old : selected) if (dist(old.point, candidate.point) < std::min(50.0, target.c.r * .2)) close = true;
            if (!close) add(candidate);
            if (selected.size() >= 6) break;
        }
        return selected;
    }

    vector<int> scanChannels(const std::set<int>& unknown, int first) const {
        vector<int> channels(unknown.begin(), unknown.end());
        auto start = std::find(channels.begin(), channels.end(), first);
        if (start != channels.end()) std::rotate(channels.begin(), start, channels.end());
        return channels;
    }

    bool unifiedDispatch(std::set<int>& unknown, vector<bool>& visitedStations) {
        const bool hasSearch = !unknown.empty() && targets.size() < 16 && scanned < static_cast<int>(search.size());
        vector<RouteGroup> groups;
        vector<Point> anchors{s.pos}, future;
        std::map<int, Point> stationPoints;
        if (hasSearch) for (size_t i = 0; i < search.size(); ++i)
            if (!visitedStations[i]) stationPoints[static_cast<int>(i)] = search[i];
        vector<int> stationOrder = route(s.pos, stationPoints);
        for (int i : stationOrder) { future.push_back(search[i]); anchors.push_back(search[i]); }
        std::map<int, Point> targetCenters;
        for (const auto& item : targets)
            if (item.second.status != "cleared") targetCenters[item.first] = item.second.c.c;
        vector<int> targetOrder = route(s.pos, targetCenters);
        for (size_t i = 0; i < std::min<size_t>(hasSearch ? 5 : 8, targetOrder.size()); ++i)
            anchors.push_back(targetCenters.at(targetOrder[i]));
        for (int i : stationOrder) {
            RouteGroup group;
            group.id = -1 - i;
            vector<int> firsts{*unknown.begin()};
            if (unknown.count(s.current) && s.current != firsts.front()) firsts.push_back(s.current);
            for (int first : firsts) {
                vector<int> channels = scanChannels(unknown, first);
                group.options.push_back({search[i], search[i], channels.front(),
                    5.0 * channels.size() + channels.size() - 1, channels.back()});
            }
            groups.push_back(std::move(group));
        }
        const int requiredMask = (1 << groups.size()) - 1;
        vector<pair<double, RouteGroup>> ranked;
        for (const auto& item : targets) {
            const Target& target = item.second;
            if (target.status == "cleared" || (hasSearch && actionsSinceSearch >= 8)) continue;
            RouteGroup group;
            group.id = target.channel;
            double priority = 1e100;
            if (target.status == "located") {
                for (Point point : clear_region::candidates(target.region, target.c, anchors)) {
                    bool duplicate = false;
                    for (const auto& old : group.options) if (dist(point, old.entry) < .1) duplicate = true;
                    if (duplicate || group.options.size() == 10) continue;
                    double service = 5;
                    if (hasSearch) service -= std::min(100.0, remainingCost(target.c, future.back()) + 5);
                    group.options.push_back({point, point, 0, service, 0});
                    priority = std::min(priority, dist(s.pos, point) / 5 + service);
                }
            } else {
                vector<CandidatePrediction> options = taskCandidates(target, anchors);
                for (const auto& option : options) {
                    double service = 10 + option.prediction.remainingTime;
                    if (hasSearch) {
                        double utility = detourGain(target, option.point, future, 0);
                        service = 13 - std::max(0.0, std::min(90.0, utility));
                    }
                    // After coverage, rank a predicted completion of each target;
                    // commit only its first reading and replan from the real state.
                    Point predictedExit = hasSearch ? option.point : target.c.c;
                    group.options.push_back({option.point, predictedExit, target.channel, service, target.channel});
                    priority = std::min(priority, dist(s.pos, option.point) / 5 + service);
                }
            }
            if (!group.options.empty()) ranked.push_back({priority, std::move(group)});
        }
        std::sort(ranked.begin(), ranked.end(), [&](const pair<double, RouteGroup>& a, const pair<double, RouteGroup>& b) {
            if (!hasSearch)
                return std::find(targetOrder.begin(), targetOrder.end(), a.second.id) <
                       std::find(targetOrder.begin(), targetOrder.end(), b.second.id);
            return a.first != b.first ? a.first < b.first : a.second.id < b.second.id;
        });
        size_t count = std::min<size_t>(hasSearch ? 4 : 7, 10 - groups.size());
        for (size_t i = 0; i < std::min(count, ranked.size()); ++i) groups.push_back(std::move(ranked[i].second));
        if (groups.empty()) return false;
        Point tail;
        bool hasTail = !hasSearch && ranked.size() > count;
        if (hasTail) tail = targetCenters.at(ranked[count].second.id);
        RouteDecision decision = solveNeighborhoodRoute(s.pos, s.current, groups, hasSearch,
                                                        hasTail ? &tail : nullptr, 0, hasSearch ? requiredMask : -1);
        if (decision.group < 0) throw std::runtime_error("No feasible unified route");
        lastWindow = groups;
        lastDecision = decision;
        const RouteGroup& selected = groups[decision.group];
        const RouteOption& option = selected.options[decision.option];
        ++unifiedDispatches;
        unifiedMaxWindow = std::max(unifiedMaxWindow, static_cast<int>(groups.size()));
        if (hasSearch && groups.size() > stationOrder.size()) ++unifiedMixedWindows;
        Json tasks = Json::array(), sequence = Json::array();
        bool s2Window = false, regionWindow = false, beforeSearch = true;
        int targetsBeforeSearch = 0, visitedMask = 0;
        for (const RouteGroup& group : groups) {
            string type = "search";
            if (group.id > 0) {
                const Target& target = targets.at(group.id);
                type = target.status == "located" ? "clear" :
                       (!target.secondRegion.empty() ? "s2_region" : "source_region");
            }
            s2Window = s2Window || type == "s2_region";
            regionWindow = regionWindow || type == "source_region";
            Json candidates = Json::array();
            for (const RouteOption& candidate : group.options)
                candidates.push_back({{"entry", {candidate.entry.x, candidate.entry.y}},
                    {"estimated_exit", {candidate.exit.x, candidate.exit.y}},
                    {"estimated_service_s", candidate.service}, {"channel", candidate.channel},
                    {"end_channel", candidate.endChannel}});
            tasks.push_back({{"task", group.id}, {"type", type}, {"candidates", candidates}});
        }
        for (const auto& choice : decision.choices) {
            if (visitedMask & (1 << choice.first)) throw std::runtime_error("Duplicate task in unified route");
            visitedMask |= 1 << choice.first;
            int id = groups[choice.first].id;
            if (id < 0) beforeSearch = false;
            else if (beforeSearch) ++targetsBeforeSearch;
            sequence.push_back({{"task", id}, {"option", choice.second}});
        }
        if (hasSearch && (visitedMask & requiredMask) != requiredMask)
            throw std::runtime_error("Unified route omitted a mandatory search station");
        if (s2Window) ++unifiedS2Windows;
        if (regionWindow) ++unifiedRegionWindows;
        if (hasSearch && targetsBeforeSearch >= 2) ++unifiedMultiBeforeSearch;
        s.record({{"kind", "unified_route_plan"}, {"task", selected.id},
            {"candidate", {{"x", option.entry.x}, {"y", option.entry.y}}},
            {"window_tasks", groups.size()}, {"search_tasks", stationOrder.size()},
            {"estimated_route_cost_s", decision.cost}, {"selected_tasks", decision.visits},
            {"cost_model", hasSearch ? "information_gain" : "predicted_completion"},
            {"tasks", tasks}, {"sequence", sequence}});
        if (selected.id < 0) {
            const size_t index = static_cast<size_t>(-1 - selected.id);
            if (visitedStations[index]) throw std::runtime_error("Repeated search station");
            s.phase = "search";
            vector<int> channels = scanChannels(unknown, option.channel);
            for (int channel : channels) {
                if (targets.count(channel)) { unknown.erase(channel); continue; }
                if (observe(search[index], channel)) unknown.erase(channel);
                if (targets.size() == 16) break;
            }
            visitedStations[index] = true;
            stationVisitOrder.push_back(static_cast<int>(index));
            ++scanned;
            ++unifiedSearchChoices;
            actionsSinceSearch = 0;
            shareMeasurements(s.pos);
            if (!quiet) std::cout << "search " << scanned << "/7; found=" << targets.size() << '\n';
        } else {
            Target& target = targets.at(selected.id);
            if (hasSearch && target.status == "located") ++unifiedClearsDuringSearch;
            if (target.status == "detected" && target.obs.size() >= 2) ++unifiedRefinements;
            ++unifiedLocalChoices;
            ++actionsSinceSearch;
            s.phase = "localization";
            executeUnifiedTarget(target, option.entry);
            shareMeasurements(s.pos);
        }
        return true;
    }

    void shareMeasurements(Point station) {
        string previousPhase = s.phase;
        s.phase = "shared";
        for (auto& item : targets) {
            Target& target = item.second;
            if (target.status != "cleared" && !target.region.empty() && farthest(target, station) <= 19.9)
                clearTarget(target, station);
        }
        vector<pair<double, int>> ranked;
        for (const auto& item : targets) {
            const Target& target = item.second;
            if (target.status != "detected" || measuredAt(target.channel, station)) continue;
            Prediction predicted = predict(target, station);
            if (!predicted.valid || predicted.receptionFraction < .45 || predicted.radius > .8 * target.c.r)
                continue;
            double saved = refinementCost(target.c.r) - refinementCost(predicted.radius) - 7;
            if (saved > 10) ranked.push_back({saved, target.channel});
        }
        std::sort(ranked.begin(), ranked.end(), [](const pair<double, int>& a, const pair<double, int>& b) {
            return a.first != b.first ? a.first > b.first : a.second < b.second;
        });
        if (ranked.size() > 4) ranked.resize(4);
        // Preserve rank for selection, then reuse the current channel when possible.
        auto current = std::find_if(ranked.begin(), ranked.end(), [&](const pair<double, int>& item) {
            return item.second == s.current;
        });
        if (current != ranked.end()) std::rotate(ranked.begin(), current, current + 1);
        for (const auto& item : ranked) {
            Target& target = targets.at(item.second);
            ++sharedMeasures;
            if (!observe(station, target.channel)) { ++sharedMisses; continue; }
            if (target.status != "detected") ++sharedLocations;
            if (target.status != "cleared" && farthest(target, station) <= 19.9) clearTarget(target, station);
        }
        s.phase = previousPhase;
    }


    Json diagnostics() const {
        Json details = Json::object();
        for (const auto& item : targets) {
            const auto& t = item.second;
            details[std::to_string(item.first)] = {
                {"discovered_time_s", t.discoveredTime}, {"located_time_s", t.locatedTime},
                {"cleared_time_s", t.clearedTime}, {"bearings", t.obs.size()},
                {"probe_attempted", t.probeAttempted}, {"failed_probe_positions", Json::array()}};
            for (Point point : t.failedClearPoints)
                details[std::to_string(item.first)]["failed_probe_positions"].push_back({point.x, point.y});
        }
        return {{"strategy", "optimized"}, {"version", PLANNER_VERSION}, {"shared_measurements", sharedMeasures},
                {"one_read_limit_m", one_read_region::limit()}, {"one_read_regions", oneReadRegions},
                {"one_read_candidates", oneReadCandidates}, {"one_read_measurements", oneReadMeasures},
                {"one_read_off_center", oneReadOffCenter}, {"one_read_failures", oneReadFailures},
                {"probe_attempts", probeAttempts}, {"probe_successes", probeSuccesses},
                {"probe_failures", probeFailures},
                {"certified_clear_failures", s.failed - probeFailures},
                {"area_threshold_m2", area_region::threshold()}, {"area_grid_spacing_m", area_region::spacing()},
                {"area_tasks_created", areaCreated}, {"area_tasks_retired", areaRetired},
                {"area_measurements", areaMeasures}, {"area_no_signal", areaMisses},
                {"unified_dispatches", unifiedDispatches}, {"unified_search_choices", unifiedSearchChoices},
                {"unified_local_choices", unifiedLocalChoices},
                {"unified_mixed_windows", unifiedMixedWindows}, {"unified_refinements", unifiedRefinements},
                {"unified_clears_during_search", unifiedClearsDuringSearch}, {"unified_max_window", unifiedMaxWindow},
                {"unified_guaranteed_misses", unifiedGuaranteedMisses}, {"station_visit_order", stationVisitOrder},
                {"unified_s2_windows", unifiedS2Windows}, {"unified_region_windows", unifiedRegionWindows},
                {"unified_multi_before_search", unifiedMultiBeforeSearch},
                {"shared_localizations", sharedLocations}, {"shared_no_signal", sharedMisses},
                {"homing_moves", homingMoves},
                {"center_measurements", centerMeasures}, {"contour_candidates", contourCandidates},
                {"targets", details}};
    }

    void homing(Target& target) {
        auto last = target.obs.back();
        double upper = std::min(1500.0, farthest(target, last.first));
        const double q = 1 / (2 * std::cos(ERR * PI / 180));
        for (int i = 0; i < 8; ++i) {
            double nextUpper = q * upper;
            Point next = last.first + unit(last.second) * nextUpper;
            ++homingMoves;
            // For d in [0,U], the worst distance after this step is q*U.
            target.region = disk(target.region, next, nextUpper);
            if (target.region.empty()) throw std::runtime_error("Empty homing region");
            if (nextUpper <= 19.9) {
                target.c = {next, nextUpper};
                markLocated(target);
                return;
            }
            if (!observe(next, target.channel))
                throw std::runtime_error("Guaranteed reception failed: this strategy requires omnidirectional sources");
            if (target.status == "cleared" || target.status == "located") return;
            upper = std::min(nextUpper, farthest(target, next));
            last = target.obs.back();
        }
        throw std::runtime_error("Homing contraction limit");
    }

    void run() {
        s.enter();
        std::set<int> unknown;
        for (int i = 1; i <= 20; ++i) unknown.insert(i);
        vector<bool> visitedStations(search.size(), false);
        // Rebuild one shared station/measurement/clear task window after each
        // committed task. Only real observations change certification regions.
        while (true) {
            bool needStation = !unknown.empty() && targets.size() < 16 &&
                               scanned < static_cast<int>(search.size());
            if (!needStation && done.size() >= targets.size()) break;
            if (!unifiedDispatch(unknown, visitedStations))
                throw std::runtime_error("Unified scheduler has unfinished tasks but no action");
        }
        absent = unknown; // All remaining channels are absent after coverage or 16 discoveries.
        if (targets.size() < 10 || targets.size() > 16)
            throw std::runtime_error("Source count outside 10..16 after full coverage; check case mode");
        complete = done.size() == targets.size();
        s.exit();
    }
};

// Offline tests share the production planner and run only with test flags.
#include "tests/self_test.hpp"

int main(int argc, char** argv) {
    SetConsoleOutputCP(CP_UTF8);
    try {
        string robot, url = "http://127.0.0.1:2026";
        string logPath = "logs/problem3_cpp_" + std::to_string(GetTickCount64()) + ".jsonl";
        bool test = false;
        string strategy = "optimized", referencePath, reportPath;
        int cases = 100, firstSeed = 0;
        for (int i = 1; i < argc; ++i) {
            string option = argv[i];
            if (option == "--self-test") test = true;
            else if (option == "--compare") { referencePath = "comparison_1000.json"; test = true; }
            else if (option == "--help") {
                std::cout << "problem3.exe --robot-id TEAM [--url http://127.0.0.1:2026] [--log FILE]\n"
                             "problem3.exe --self-test [--cases 100] [--seed-start 0] [--report validation.json]\n"
                             "problem3.exe --compare [--cases 100] [--report comparison_v2.json]\n"
                             "problem3.exe --compare-with REFERENCE.json [--cases 100] [--report FILE]\n"
                             "Only optimized is retained. --compare uses comparison_1000.json.\n";
                return 0;
            } else if (i + 1 < argc && option == "--robot-id") robot = argv[++i];
            else if (i + 1 < argc && option == "--url") url = argv[++i];
            else if (i + 1 < argc && option == "--log") logPath = argv[++i];
            else if (i + 1 < argc && option == "--cases") cases = std::stoi(argv[++i]);
            else if (i + 1 < argc && option == "--seed-start") firstSeed = std::stoi(argv[++i]);
            else if (i + 1 < argc && option == "--strategy") strategy = argv[++i];
            else if (i + 1 < argc && option == "--compare-with") { referencePath = argv[++i]; test = true; }
            else if (i + 1 < argc && option == "--report") reportPath = argv[++i];
            else throw std::runtime_error("Unknown or incomplete option: " + option);
        }
        if (strategy != "optimized")
            throw std::runtime_error("Only optimized is retained; use --compare for historical results");
        if (test) {
            if (cases < 1 || cases > 10000) throw std::runtime_error("--cases must be in 1..10000");
            if (firstSeed < 0 || firstSeed > 1000000)
                throw std::runtime_error("--seed-start must be in 0..1000000");
            return selfTest(cases, referencePath, reportPath, firstSeed);
        }
        if (robot.empty()) throw std::runtime_error("--robot-id is required (use --self-test for offline testing)");
        WinHttpTransport transport(url);
        Session session(transport, robot, logPath);
        Planner planner(session);
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
        const bool totalKnown = planner.scanned == static_cast<int>(planner.search.size()) ||
                                planner.targets.size() == 16;
        summary.update(completionMetrics(session.cleared,
            totalKnown ? static_cast<int>(planner.targets.size()) : -1, session.vt));
        summary["discovered_sources"] = planner.targets.size();
        summary["planner"] = planner.diagnostics();
        summary["complete"] = planner.complete && error.empty();
        summary["cleared_channels"] = planner.done;
        summary["absent_channels"] = planner.absent;
        summary["search_points_visited"] = planner.scanned;
        summary["error"] = error;
        session.record({{"kind", "summary"}, {"summary", summary}});
        std::ofstream summaryFile(logPath + ".summary.json");
        summaryFile << summary.dump(2) << '\n';
        std::cout << summary.dump(2) << "\nlog: " << logPath << '\n';
        if (!error.empty()) std::cerr << "ERROR: " << error << '\n';
        printCompletionMetrics(summary);
        return error.empty() ? 0 : 2;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 2;
    }
}

