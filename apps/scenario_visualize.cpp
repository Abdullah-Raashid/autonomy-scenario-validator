#include "cli_common.h"
#include "io/scenario_loader.h"
#include "vision/renderer.h"
#include <iostream>
int main(int argc, char **argv) {
    try {
        auto values = asv::cli::ScenarioValues();
        values.insert({"--frames", "--video", "--fps"});
        const asv::cli::Args a(argc, argv, values, {"--help"});
        if (a.Has("--help")) {
            std::cout << "scenario_visualize " << asv::cli::ScenarioHelp()
                      << "  --frames DIRECTORY [--video FILE.mp4] [--fps 10]\n";
            return 0;
        }
        const auto p = asv::cli::Params(a);
        asv::Scenario s;
        std::string error;
        if (!asv::ScenarioLoader::LoadFromCsv(a.Required("--traj"), a.Get("--obs"), s, error))
            throw std::invalid_argument(error);
        auto r = asv::ValidationEngine(p).Validate(s);
        r.meta.trajectoryFile = a.Get("--traj");
        r.meta.obstaclesFile = a.Get("--obs");
        if (a.Has("--out"))
            asv::cli::Write(a.Required("--out"), r.ToJson());
        const auto ctx = asv::Prepare(s);
        asv::vision::Renderer(ctx, r).Export(a.Required("--frames"), a.Get("--video"),
                                             a.Number("--fps", 10));
        std::cout << "Exported " << ctx.frames.size() << " PNG frames"
                  << (a.Has("--video") ? " and video" : "") << "\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "scenario_visualize: " << e.what() << "\n";
        return 2;
    }
}
