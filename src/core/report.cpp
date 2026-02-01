#include "core/report.h"
#include <sstream>
#include <iomanip>

namespace asv
{

    static std::string EscapeJson(const std::string &s)
    {
        std::string out;
        out.reserve(s.size() + 8);
        for (size_t i = 0; i < s.size(); ++i)
        {
            const char c = s[i];
            switch (c)
            {
            case '\"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += c;
                break;
            }
        }
        return out;
    }

    static void JsonKV(std::ostringstream &oss, const std::string &k, const std::string &v)
    {
        oss << "\"" << EscapeJson(k) << "\": \"" << EscapeJson(v) << "\"";
    }

    static void JsonKVD(std::ostringstream &oss, const std::string &k, double v)
    {
        oss << "\"" << EscapeJson(k) << "\": " << std::fixed << std::setprecision(6) << v;
    }

    static void JsonKVI(std::ostringstream &oss, const std::string &k, int v)
    {
        oss << "\"" << EscapeJson(k) << "\": " << v;
    }

    void Report::Add(double t, const std::string &rule, const std::string &severity, const std::string &details)
    {
        Violation v;
        v.t = t;
        v.rule = rule;
        v.severity = severity;
        v.details = details;
        violations.push_back(v);
    }

    Summary Report::ComputeSummary() const
    {
        Summary s;
        s.totalViolations = static_cast<int>(violations.size());
        s.countsBySeverity["info"] = 0;
        s.countsBySeverity["warn"] = 0;
        s.countsBySeverity["error"] = 0;

        for (size_t i = 0; i < violations.size(); ++i)
        {
            const Violation &v = violations[i];

            s.countsBySeverity[v.severity] += 1;
            s.countsByRule[v.rule] += 1;

            if (s.firstViolationT < 0.0 || v.t < s.firstViolationT)
                s.firstViolationT = v.t;
            if (s.lastViolationT < 0.0 || v.t > s.lastViolationT)
                s.lastViolationT = v.t;
        }

        // "Applied Intuition-like" overall result:
        // any error => FAIL, else any warn => WARN, else PASS
        if (s.countsBySeverity["error"] > 0)
            s.result = "FAIL";
        else if (s.countsBySeverity["warn"] > 0)
            s.result = "WARN";
        else
            s.result = "PASS";

        return s;
    }

    std::string Report::ToJson() const
    {
        const Summary summary = ComputeSummary();

        std::ostringstream oss;
        oss << "{\n";

        // scenario meta
        oss << "  \"scenario\": {\n";
        oss << "    ";
        JsonKV(oss, "trajectory_file", meta.trajectoryFile);
        oss << ",\n    ";
        JsonKV(oss, "obstacles_file", meta.obstaclesFile);
        oss << ",\n";

        oss << "    \"params\": {\n";
        oss << "      ";
        JsonKVD(oss, "max_speed_mps", meta.params.maxSpeedMps);
        oss << ",\n      ";
        JsonKVD(oss, "max_accel_mps2", meta.params.maxAccelMps2);
        oss << ",\n      ";
        JsonKVD(oss, "lane_half_width_m", meta.params.laneHalfWidthM);
        oss << ",\n      \"ego_box\": { ";
        JsonKVD(oss, "w", meta.params.egoBox.w);
        oss << ", ";
        JsonKVD(oss, "h", meta.params.egoBox.h);
        oss << " }\n";
        oss << "    },\n";

        oss << "    \"time_range_s\": { ";
        JsonKVD(oss, "start", meta.scenarioStartT);
        oss << ", ";
        JsonKVD(oss, "end", meta.scenarioEndT);
        oss << " }\n";

        oss << "  },\n";

        // summary
        oss << "  \"summary\": {\n";
        oss << "    ";
        JsonKV(oss, "result", summary.result);
        oss << ",\n    ";
        JsonKVI(oss, "total_violations", summary.totalViolations);
        oss << ",\n    ";
        JsonKVD(oss, "first_violation_s", summary.firstViolationT);
        oss << ",\n    ";
        JsonKVD(oss, "last_violation_s", summary.lastViolationT);
        oss << ",\n";

        oss << "    \"counts_by_severity\": {";
        bool first = true;
        for (std::map<std::string, int>::const_iterator it = summary.countsBySeverity.begin();
             it != summary.countsBySeverity.end(); ++it)
        {
            if (!first)
                oss << ", ";
            first = false;
            oss << "\"" << EscapeJson(it->first) << "\": " << it->second;
        }
        oss << "},\n";

        oss << "    \"counts_by_rule\": {";
        first = true;
        for (std::map<std::string, int>::const_iterator it = summary.countsByRule.begin();
             it != summary.countsByRule.end(); ++it)
        {
            if (!first)
                oss << ", ";
            first = false;
            oss << "\"" << EscapeJson(it->first) << "\": " << it->second;
        }
        oss << "}\n";

        oss << "  },\n";

        // violations list
        oss << "  \"violations\": [\n";
        for (size_t i = 0; i < violations.size(); ++i)
        {
            const Violation &v = violations[i];
            oss << "    {";
            JsonKVD(oss, "t", v.t);
            oss << ", ";
            JsonKV(oss, "rule", v.rule);
            oss << ", ";
            JsonKV(oss, "severity", v.severity);
            oss << ", ";
            JsonKV(oss, "details", v.details);
            oss << "}";
            if (i + 1 < violations.size())
                oss << ",";
            oss << "\n";
        }
        oss << "  ]\n";

        oss << "}\n";
        return oss.str();
    }

} // namespace asv
