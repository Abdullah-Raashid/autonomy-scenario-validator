#pragma once
#include <string>
#include <vector>
#include <map>

namespace asv
{

    struct Violation
    {
        double t = 0.0;
        std::string rule;
        std::string severity; // "info" | "warn" | "error"
        std::string details;
    };

    struct EgoBox
    {
        double w = 0.0;
        double h = 0.0;
    };

    struct RunParams
    {
        double maxSpeedMps = 0.0;
        double maxAccelMps2 = 0.0;
        double laneHalfWidthM = 0.0;
        EgoBox egoBox;
    };

    struct ScenarioMeta
    {
        std::string trajectoryFile;
        std::string obstaclesFile;
        RunParams params;
        double scenarioStartT = 0.0;
        double scenarioEndT = 0.0;
    };

    struct Summary
    {
        std::string result; // "PASS" | "WARN" | "FAIL"
        int totalViolations = 0;
        double firstViolationT = -1.0;
        double lastViolationT = -1.0;
        std::map<std::string, int> countsBySeverity;
        std::map<std::string, int> countsByRule;
    };

    struct Report
    {
        ScenarioMeta meta;
        std::vector<Violation> violations;

        void Add(double t, const std::string &rule, const std::string &severity, const std::string &details);
        Summary ComputeSummary() const;
        std::string ToJson() const;
    };

} // namespace asv
