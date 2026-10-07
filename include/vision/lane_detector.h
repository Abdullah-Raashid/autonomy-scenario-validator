#pragma once
#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

namespace asv::vision {
struct LaneLine {
    double a = 0.0, b = 0.0; // x = a*y + b in IMAGE PIXELS
    double X(double y) const { return a * y + b; }
};
struct LaneConfig {
    double cannyLow = 50, cannyHigh = 140;
    double roiTop = 0.55;
    double minAbsDxDy = 0.15, maxAbsDxDy = 1.5;
    int houghVotes = 25;
    double minLengthFraction = 0.08, maxGapFraction = 0.04;
};
struct LaneResult {
    cv::Mat gray, blurred, edges, roiEdges, overlay;
    std::vector<cv::Vec4i> candidates;
    std::optional<LaneLine> left, right;
};
LaneResult DetectLanes(const cv::Mat &image, const LaneConfig &config = {});
cv::Mat SyntheticRoad(int width = 960, int height = 540, std::uint64_t seed = 42);
cv::Mat DebugPanel(const cv::Mat &input, const LaneResult &result);
} // namespace asv::vision
