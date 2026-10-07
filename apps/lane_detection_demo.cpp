#include "cli_common.h"
#include "vision/lane_detector.h"
#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
int main(int argc, char **argv) {
    try {
        const asv::cli::Args a(argc, argv, {"--input", "--output"}, {"--help", "--synthetic"});
        if (a.Has("--help")) {
            std::cout << "lane_detection_demo (--synthetic | --input IMAGE) --output DIRECTORY\n";
            return 0;
        }
        if (a.Has("--synthetic") == a.Has("--input"))
            throw std::invalid_argument("choose exactly one of --synthetic or --input");
        const auto directory = std::filesystem::path(a.Required("--output"));
        const auto input =
            a.Has("--synthetic") ? asv::vision::SyntheticRoad() : cv::imread(a.Required("--input"));
        const auto r = asv::vision::DetectLanes(input);
        std::filesystem::create_directories(directory);
        const std::pair<const char *, cv::Mat> images[] = {
            {"input.png", input},
            {"gray.png", r.gray},
            {"blurred.png", r.blurred},
            {"edges.png", r.edges},
            {"roi_edges.png", r.roiEdges},
            {"overlay.png", r.overlay},
            {"pipeline.png", asv::vision::DebugPanel(input, r)}};
        for (const auto &[name, image] : images)
            if (!cv::imwrite((directory / name).string(), image))
                throw std::runtime_error("cannot write " + (directory / name).string());
        std::cout << "Classical Vision Lane Detection Demo\nPixel-space estimates: left="
                  << (r.left ? "detected" : "unavailable")
                  << ", right=" << (r.right ? "detected" : "unavailable")
                  << ", Hough segments=" << r.candidates.size() << "\n";
        if (r.left)
            std::cout << "left: x = " << r.left->a << " * y + " << r.left->b << "\n";
        if (r.right)
            std::cout << "right: x = " << r.right->a << " * y + " << r.right->b << "\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "lane_detection_demo: " << e.what() << "\n";
        return 2;
    }
}
