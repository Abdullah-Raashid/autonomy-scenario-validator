#pragma once
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace asv {
enum class Severity { Info, Warning, Error };
const char *ToString(Severity severity);
struct Violation {
    double t = 0.0;
    std::string rule;
    Severity severity = Severity::Info;
    std::string details;
    std::optional<std::size_t> sampleIndex;
};
struct EgoBox {
    double length = 4.5, width = 2.0;
};
struct RunParams {
    double maxSpeedMps = 15.0, maxAccelMps2 = 3.0, maxJerkMps3 = 5.0;
    double laneHalfWidthM = 1.8, nearDistanceM = 1.0, minTtcS = 2.0;
    EgoBox egoBox;
};
struct ScenarioMeta {
    std::string trajectoryFile, obstaclesFile;
    RunParams params;
    double scenarioStartT = 0.0, scenarioEndT = 0.0;
    std::size_t sampleCount = 0;
};
struct Metrics {
    std::optional<double> maxSpeedMps, maxAccelMps2, maxJerkMps3;
    std::optional<double> minObstacleDistanceM, minLaneMarginM, minTtcS;
};
struct Summary {
    std::string result;
    std::size_t totalViolations = 0;
    std::optional<double> firstViolationT, lastViolationT;
    std::map<std::string, std::size_t> countsBySeverity, countsByRule;
};
struct Report {
    ScenarioMeta meta;
    Metrics metrics;
    std::vector<Violation> violations;
    void Add(double t, const std::string &rule, Severity severity, std::string details,
             std::optional<std::size_t> sampleIndex = std::nullopt);
    Summary ComputeSummary() const;
    std::string ToJson() const;
};
} // namespace asv
