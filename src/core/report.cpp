#include "core/report.h"
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace asv {
const char *ToString(Severity s) {
    switch (s) {
    case Severity::Info:
        return "info";
    case Severity::Warning:
        return "warn";
    case Severity::Error:
        return "error";
    }
    throw std::invalid_argument("invalid severity");
}
namespace {
std::string Quote(const std::string &value) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << '"';
    for (std::size_t i = 0; i < value.size(); ++i) {
        const auto c = static_cast<unsigned char>(value[i]);
        if (c >= 0x80) {
            // Preserve valid UTF-8; reject malformed byte sequences instead of changing IDs.
            const std::size_t length = c >= 0xc2 && c <= 0xdf   ? 2
                                       : c >= 0xe0 && c <= 0xef ? 3
                                       : c >= 0xf0 && c <= 0xf4 ? 4
                                                                : 0;
            if (length == 0 || i + length > value.size())
                throw std::invalid_argument("invalid UTF-8 in report string");
            unsigned codepoint = c & (0x7f >> length);
            for (std::size_t j = 1; j < length; ++j) {
                const auto next = static_cast<unsigned char>(value[i + j]);
                if ((next & 0xc0) != 0x80)
                    throw std::invalid_argument("invalid UTF-8 in report string");
                codepoint = (codepoint << 6) | (next & 0x3f);
            }
            if ((length == 3 && codepoint < 0x800) || (length == 4 && codepoint < 0x10000) ||
                (codepoint >= 0xd800 && codepoint <= 0xdfff) || codepoint > 0x10ffff)
                throw std::invalid_argument("invalid UTF-8 in report string");
            out << value.substr(i, length);
            i += length - 1;
            continue;
        }
        switch (c) {
        case '"':
            out << "\\\"";
            break;
        case '\\':
            out << "\\\\";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            if (c < 0x20)
                out << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<int>(c) << std::dec;
            else
                out << static_cast<char>(c);
        }
    }
    out << '"';
    return out.str();
}
void Number(std::ostream &out, double v) {
    if (!std::isfinite(v))
        throw std::invalid_argument("cannot serialize nonfinite number");
    out << v;
}
void Optional(std::ostream &out, std::optional<double> v) {
    if (v)
        Number(out, *v);
    else
        out << "null";
}
void Counts(std::ostream &out, const std::map<std::string, std::size_t> &counts) {
    out << '{';
    bool first = true;
    for (const auto &[key, count] : counts) {
        if (!first)
            out << ", ";
        first = false;
        out << Quote(key) << ": " << count;
    }
    out << '}';
}
} // namespace
void Report::Add(double t, const std::string &rule, Severity severity, std::string details,
                 std::optional<std::size_t> index) {
    if (!std::isfinite(t) || t < 0.0)
        throw std::invalid_argument("invalid violation timestamp");
    ToString(severity);
    violations.push_back({t, rule, severity, std::move(details), index});
}
Summary Report::ComputeSummary() const {
    Summary s;
    s.totalViolations = violations.size();
    s.countsBySeverity = {{"info", 0}, {"warn", 0}, {"error", 0}};
    for (const auto &v : violations) {
        ++s.countsBySeverity[ToString(v.severity)];
        ++s.countsByRule[v.rule];
        if (!s.firstViolationT || v.t < *s.firstViolationT)
            s.firstViolationT = v.t;
        if (!s.lastViolationT || v.t > *s.lastViolationT)
            s.lastViolationT = v.t;
    }
    s.result = s.countsBySeverity["error"] ? "FAIL" : s.countsBySeverity["warn"] ? "WARN" : "PASS";
    return s;
}
std::string Report::ToJson() const {
    const auto s = ComputeSummary();
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17);
    out << "{\n  \"schema_version\": 2,\n  \"scenario\": {\n    \"trajectory_file\": "
        << Quote(meta.trajectoryFile) << ",\n    \"obstacles_file\": " << Quote(meta.obstaclesFile)
        << ",\n    \"sample_count\": " << meta.sampleCount
        << ",\n    \"time_range_s\": {\"start\": ";
    Number(out, meta.scenarioStartT);
    out << ", \"end\": ";
    Number(out, meta.scenarioEndT);
    out << "},\n    \"params\": {";
    const auto &p = meta.params;
    const std::pair<const char *, double> params[] = {
        {"max_speed_mps", p.maxSpeedMps},     {"max_accel_mps2", p.maxAccelMps2},
        {"max_jerk_mps3", p.maxJerkMps3},     {"lane_half_width_m", p.laneHalfWidthM},
        {"near_distance_m", p.nearDistanceM}, {"min_ttc_s", p.minTtcS},
        {"ego_length_m", p.egoBox.length},    {"ego_width_m", p.egoBox.width}};
    bool first = true;
    for (const auto &[key, value] : params) {
        if (!first)
            out << ", ";
        first = false;
        out << Quote(key) << ": ";
        Number(out, value);
    }
    out << "}\n  },\n  \"summary\": {\n    \"result\": " << Quote(s.result)
        << ",\n    \"total_violations\": " << s.totalViolations << ",\n    \"first_violation_s\": ";
    Optional(out, s.firstViolationT);
    out << ",\n    \"last_violation_s\": ";
    Optional(out, s.lastViolationT);
    out << ",\n    \"counts_by_severity\": ";
    Counts(out, s.countsBySeverity);
    out << ",\n    \"counts_by_rule\": ";
    Counts(out, s.countsByRule);
    out << "\n  },\n  \"metrics\": {";
    const std::pair<const char *, std::optional<double>> metricEntries[] = {
        {"max_speed_mps", metrics.maxSpeedMps},
        {"max_accel_mps2", metrics.maxAccelMps2},
        {"max_jerk_mps3", metrics.maxJerkMps3},
        {"min_obstacle_distance_m", metrics.minObstacleDistanceM},
        {"min_lane_margin_m", metrics.minLaneMarginM},
        {"min_ttc_s", metrics.minTtcS}};
    first = true;
    for (const auto &[key, value] : metricEntries) {
        if (!first)
            out << ", ";
        first = false;
        out << Quote(key) << ": ";
        Optional(out, value);
    }
    out << "},\n  \"violations\": [";
    for (std::size_t i = 0; i < violations.size(); ++i) {
        const auto &v = violations[i];
        if (i)
            out << ',';
        out << "\n    {\"t\": ";
        Number(out, v.t);
        out << ", \"sample_index\": ";
        if (v.sampleIndex)
            out << *v.sampleIndex;
        else
            out << "null";
        out << ", \"rule\": " << Quote(v.rule) << ", \"severity\": " << Quote(ToString(v.severity))
            << ", \"details\": " << Quote(v.details) << '}';
    }
    out << "\n  ]\n}\n";
    return out.str();
}
} // namespace asv
