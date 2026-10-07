#include "vision/lane_detector.h"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace asv::vision {
namespace {
// Segment-length weighted least squares of endpoints: x = a*y + b.
struct Fit {
    double weight = 0, sy = 0, sx = 0, syy = 0, syx = 0;
    void Add(double x, double y, double w) {
        weight += w;
        sy += w * y;
        sx += w * x;
        syy += w * y * y;
        syx += w * y * x;
    }
    std::optional<LaneLine> Line() const {
        const double determinant = weight * syy - sy * sy;
        if (weight == 0 || determinant <= 0)
            return std::nullopt;
        const double a = (weight * syx - sy * sx) / determinant;
        return LaneLine{a, (sx - a * sy) / weight};
    }
};
cv::Point Point(double x, double y) { return {cvRound(x), cvRound(y)}; }
void DrawLine(cv::Mat &image, const LaneLine &line, double y0, double y1, const cv::Scalar &color) {
    cv::line(image, Point(line.X(y0), y0), Point(line.X(y1), y1), color, 5, cv::LINE_AA);
}
} // namespace
LaneResult DetectLanes(const cv::Mat &image, const LaneConfig &c) {
    if (image.empty() || (image.type() != CV_8UC3 && image.type() != CV_8UC1) || image.cols < 32 ||
        image.rows < 32)
        throw std::invalid_argument(
            "lane detection requires an 8-bit gray/BGR image at least 32x32");
    if (!std::isfinite(c.cannyLow) || !std::isfinite(c.cannyHigh) || c.cannyLow < 0 ||
        c.cannyHigh <= c.cannyLow || !std::isfinite(c.roiTop) || c.roiTop <= 0 || c.roiTop >= 1 ||
        c.houghVotes <= 0 || !std::isfinite(c.minAbsDxDy) || !std::isfinite(c.maxAbsDxDy) ||
        c.minAbsDxDy <= 0 || c.maxAbsDxDy <= c.minAbsDxDy || !std::isfinite(c.minLengthFraction) ||
        !std::isfinite(c.maxGapFraction) || c.minLengthFraction <= 0 || c.minLengthFraction > 1 ||
        c.maxGapFraction < 0 || c.maxGapFraction > 1)
        throw std::invalid_argument("invalid lane configuration");
    LaneResult r;
    if (image.channels() == 3) {
        cv::cvtColor(image, r.gray, cv::COLOR_BGR2GRAY);
        r.overlay = image.clone();
    } else {
        r.gray = image.clone();
        cv::cvtColor(image, r.overlay, cv::COLOR_GRAY2BGR);
    }
    cv::GaussianBlur(r.gray, r.blurred, {5, 5}, 1.3);
    cv::Canny(r.blurred, r.edges, c.cannyLow, c.cannyHigh, 3, true);
    const double w = image.cols - 1, h = image.rows - 1, top = c.roiTop * h;
    const std::vector<cv::Point> roi = {Point(0.08 * w, h), Point(0.42 * w, top),
                                        Point(0.58 * w, top), Point(0.92 * w, h)};
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::fillConvexPoly(mask, roi, cv::Scalar(255));
    cv::bitwise_and(r.edges, mask, r.roiEdges);
    cv::HoughLinesP(r.roiEdges, r.candidates, 1, CV_PI / 180, c.houghVotes,
                    c.minLengthFraction * image.rows, c.maxGapFraction * image.rows);
    Fit left, right;
    for (const auto &s : r.candidates) {
        const double dx = s[2] - s[0], dy = s[3] - s[1];
        if (std::abs(dy) < 1)
            continue;
        const double a = dx / dy;
        if (std::abs(a) < c.minAbsDxDy || std::abs(a) > c.maxAbsDxDy)
            continue;
        const double midX = (s[0] + s[2]) * 0.5;
        if ((a < 0 && midX >= w * 0.5) || (a > 0 && midX <= w * 0.5))
            continue;
        const double b = s[0] - a * s[1], bottomX = a * h + b;
        if (bottomX < 0 || bottomX > w || (a < 0 && bottomX >= w * 0.5) ||
            (a > 0 && bottomX <= w * 0.5))
            continue;
        auto &fit = a < 0 ? left : right;
        const double weight = std::hypot(dx, dy);
        fit.Add(s[0], s[1], weight);
        fit.Add(s[2], s[3], weight);
    }
    r.left = left.Line();
    r.right = right.Line();
    if (r.left && r.right && (r.left->X(top) >= r.right->X(top) || r.left->X(h) >= r.right->X(h))) {
        r.left.reset();
        r.right.reset();
    }
    cv::polylines(r.overlay, roi, true, cv::Scalar(110, 110, 110), 1, cv::LINE_AA);
    if (r.left)
        DrawLine(r.overlay, *r.left, top, h, {80, 220, 80});
    if (r.right)
        DrawLine(r.overlay, *r.right, top, h, {40, 180, 255});
    return r;
}
cv::Mat SyntheticRoad(int width, int height, std::uint64_t seed) {
    if (width < 64 || height < 64 || width > 8192 || height > 8192)
        throw std::invalid_argument("synthetic dimensions must be in [64,8192]");
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(66, 65, 62));
    const double w = width - 1, h = height - 1;
    cv::rectangle(image, {0, 0, width, cvRound(0.53 * h)}, cv::Scalar(155, 115, 70), cv::FILLED);
    const int thickness = std::max(2, cvRound(width * 0.008));
    cv::line(image, Point(.20 * w, h), Point(.44 * w, .58 * h), cv::Scalar(245, 245, 245),
             thickness, cv::LINE_AA);
    cv::line(image, Point(.80 * w, h), Point(.56 * w, .58 * h), cv::Scalar(60, 230, 245), thickness,
             cv::LINE_AA);
    // Distractors: horizontal marks and an edge above the ROI.
    cv::line(image, Point(.35 * w, .85 * h), Point(.65 * w, .85 * h), cv::Scalar(95, 95, 95), 2);
    cv::line(image, Point(.1 * w, .2 * h), Point(.9 * w, .2 * h), cv::Scalar(220, 220, 220), 3);
    cv::RNG random(seed);
    cv::Mat noise(image.size(), CV_16SC3);
    random.fill(noise, cv::RNG::NORMAL, 0, 3);
    cv::Mat wide;
    image.convertTo(wide, CV_16SC3);
    cv::add(wide, noise, wide);
    wide.convertTo(image, CV_8UC3);
    return image;
}
cv::Mat DebugPanel(const cv::Mat &input, const LaneResult &r) {
    cv::Mat panel(3 * 300, 2 * 480, CV_8UC3, cv::Scalar(20, 24, 30));
    const cv::Mat stages[] = {input, r.gray, r.blurred, r.edges, r.roiEdges, r.overlay};
    const char *labels[] = {"01  INPUT / SYNTHETIC ROAD", "02  GRAYSCALE",
                            "03  GAUSSIAN FILTER",        "04  CANNY EDGES",
                            "05  REGION OF INTEREST",     "06  HOUGH + GEOMETRIC FIT"};
    for (int i = 0; i < 6; ++i) {
        cv::Mat color, small;
        if (stages[i].channels() == 1)
            cv::cvtColor(stages[i], color, cv::COLOR_GRAY2BGR);
        else
            color = stages[i];
        cv::resize(color, small, {480, 270});
        small.copyTo(panel(cv::Rect((i % 2) * 480, (i / 2) * 300 + 30, 480, 270)));
        cv::putText(panel, labels[i], {(i % 2) * 480 + 12, (i / 2) * 300 + 21},
                    cv::FONT_HERSHEY_SIMPLEX, .48, {220, 228, 235}, 1, cv::LINE_AA);
    }
    return panel;
}
} // namespace asv::vision
