#include "cli_common.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {
std::size_t Count(const asv::cli::Args &a, const std::string &key, std::size_t fallback,
                  std::size_t maximum) {
    const double v = a.Number(key, static_cast<double>(fallback));
    if (v < 1 || v > static_cast<double>(maximum) || std::floor(v) != v)
        throw std::invalid_argument("invalid count: " + key);
    return static_cast<std::size_t>(v);
}
} // namespace
int main(int argc, char **argv) {
    try {
        const asv::cli::Args a(argc, argv, {"--samples", "--obstacles", "--repeats"},
                               {"--help", "--stress"});
        if (a.Has("--help")) {
            std::cout
                << "asv_benchmark [--samples 10000] [--obstacles 20] [--repeats 5] [--stress]\n";
            return 0;
        }
        const auto n = Count(a, "--samples", 10000, 10000000),
                   m = Count(a, "--obstacles", 20, 10000), repeats = Count(a, "--repeats", 5, 100);
        asv::Scenario s;
        s.ego.points.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            const double t = static_cast<double>(i) * 0.05;
            s.ego.points.push_back({t, t * 10.0, 0.0, 0.0});
        }
        for (std::size_t j = 0; j < m; ++j)
            s.obstacles.items.push_back({"obstacle_" + std::to_string(j),
                                         {a.Has("--stress") ? static_cast<double>(j) * 0.25
                                                            : 20.0 + static_cast<double>(j) * 10,
                                          a.Has("--stress") ? 0.0 : 8.0, 4.0, 2.0},
                                         0.1,
                                         {10.0, 0.0}});
        asv::ValidationEngine engine;
        auto warmup = engine.Validate(s);
        const auto expected = warmup.violations.size();
        std::vector<double> times;
        std::size_t checksum = 0;
        for (std::size_t run = 0; run < repeats; ++run) {
            const auto start = std::chrono::steady_clock::now();
            const auto report = engine.Validate(s);
            const auto end = std::chrono::steady_clock::now();
            if (report.violations.size() != expected)
                throw std::runtime_error("nondeterministic violation count");
            checksum += report.violations.size() + report.meta.sampleCount;
            times.push_back(std::chrono::duration<double>(end - start).count());
        }
        std::sort(times.begin(), times.end());
        const double median = times.size() % 2
                                  ? times[times.size() / 2]
                                  : 0.5 * (times[times.size() / 2 - 1] + times[times.size() / 2]);
        std::cout << std::setprecision(10)
                  << "samples,obstacles,validators,repeats,min_s,median_s,max_s,samples_per_s,"
                     "violations_per_run,checksum\n"
                  << n << ',' << m << ',' << engine.ValidatorCount() << ',' << repeats << ','
                  << times.front() << ',' << median << ',' << times.back() << ','
                  << static_cast<double>(n) / median << ',' << expected << ',' << checksum << '\n';
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "asv_benchmark: " << e.what() << '\n';
        return 2;
    }
}
