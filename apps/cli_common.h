#pragma once
#include "core/validation.h"
#include "io/numeric.h"
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace asv::cli {
class Args {
  public:
    Args(int argc, char **argv, std::set<std::string> values, std::set<std::string> flags) {
        const std::map<std::string, std::string> aliases = {
            {"--max_speed", "--max-speed"},
            {"--max_accel", "--max-accel"},
            {"--lane_half_width", "--lane-half-width"},
            {"--ego_w", "--ego-width"},
            {"--ego_h", "--ego-length"}};
        for (int i = 1; i < argc; ++i) {
            std::string key = argv[i];
            const auto alias = aliases.find(key);
            if (alias != aliases.end())
                key = alias->second;
            if (options_.count(key))
                throw std::invalid_argument("duplicate option: " + key);
            if (flags.count(key))
                options_[key] = "true";
            else if (values.count(key)) {
                if (i + 1 == argc || std::string(argv[i + 1]).rfind("--", 0) == 0)
                    throw std::invalid_argument("missing value: " + key);
                options_[key] = argv[++i];
            } else
                throw std::invalid_argument("unknown option: " + key);
        }
    }
    bool Has(const std::string &key) const { return options_.count(key) > 0; }
    std::string Get(const std::string &key, const std::string &fallback = {}) const {
        const auto it = options_.find(key);
        return it == options_.end() ? fallback : it->second;
    }
    double Number(const std::string &key, double fallback) const {
        return Has(key) ? ParseFiniteDouble(Get(key)) : fallback;
    }
    std::string Required(const std::string &key) const {
        if (!Has(key) || Get(key).empty())
            throw std::invalid_argument("required option: " + key);
        return Get(key);
    }

  private:
    std::map<std::string, std::string> options_;
};
inline std::set<std::string> ScenarioValues() {
    return {"--traj",
            "--obs",
            "--out",
            "--max-speed",
            "--max-accel",
            "--max-jerk",
            "--lane-half-width",
            "--ego-length",
            "--ego-width",
            "--near-distance",
            "--min-ttc"};
}
inline RunParams Params(const Args &a) {
    RunParams p;
    p.maxSpeedMps = a.Number("--max-speed", p.maxSpeedMps);
    p.maxAccelMps2 = a.Number("--max-accel", p.maxAccelMps2);
    p.maxJerkMps3 = a.Number("--max-jerk", p.maxJerkMps3);
    p.laneHalfWidthM = a.Number("--lane-half-width", p.laneHalfWidthM);
    p.egoBox.length = a.Number("--ego-length", p.egoBox.length);
    p.egoBox.width = a.Number("--ego-width", p.egoBox.width);
    p.nearDistanceM = a.Number("--near-distance", p.nearDistanceM);
    p.minTtcS = a.Number("--min-ttc", p.minTtcS);
    ValidateParams(p);
    return p;
}
inline void Write(const std::string &path, const std::string &contents) {
    std::ofstream out(path);
    out << contents;
    out.close();
    if (!out)
        throw std::runtime_error("cannot write output: " + path);
}
inline std::string ScenarioHelp() {
    return "--traj CSV [--obs CSV] [--out JSON] [--max-speed 15] [--max-accel 3] [--max-jerk 5]\n"
           "  [--lane-half-width 1.8] [--ego-length 4.5] [--ego-width 2] [--near-distance 1] "
           "[--min-ttc 2]\n";
}
} // namespace asv::cli
