#include "cli_common.h"
#include "io/scenario_loader.h"
#include <iostream>

int main(int argc, char **argv) {
    try {
        const asv::cli::Args args(argc, argv, asv::cli::ScenarioValues(),
                                  {"--help", "--fail-on-violation"});
        if (args.Has("--help")) {
            std::cout << "scenario_playback " << asv::cli::ScenarioHelp()
                      << "  [--fail-on-violation]\n";
            return 0;
        }
        const auto params = asv::cli::Params(args);
        asv::Scenario s;
        std::string error;
        if (!asv::ScenarioLoader::LoadFromCsv(args.Required("--traj"), args.Get("--obs"), s, error))
            throw std::invalid_argument(error);
        auto report = asv::ValidationEngine(params).Validate(s);
        report.meta.trajectoryFile = args.Get("--traj");
        report.meta.obstaclesFile = args.Get("--obs");
        if (args.Has("--out"))
            asv::cli::Write(args.Required("--out"), report.ToJson());
        else
            std::cout << report.ToJson();
        return args.Has("--fail-on-violation") && report.ComputeSummary().result != "PASS" ? 3 : 0;
    } catch (const std::exception &e) {
        std::cerr << "scenario_playback: " << e.what() << "\n";
        return 2;
    }
}
