#include "vision/renderer.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <locale>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace asv::vision {
namespace {
constexpr int maxGridIntervals = 32;
constexpr double baseGridSpacingM = 5.0;
int RoundPixel(double value) {
    if (!std::isfinite(value) || value <= std::numeric_limits<int>::min() ||
        value >= std::numeric_limits<int>::max())
        throw std::invalid_argument("pixel coordinate exceeds numeric range");
    return cvRound(value);
}
const cv::Scalar ink(225, 232, 240), muted(142, 154, 170), teal(180, 210, 60), amber(65, 185, 255),
    red(80, 80, 240);
void Text(cv::Mat &image, const std::string &text, cv::Point point, double size = .55,
          cv::Scalar color = ink) {
    cv::putText(image, text, point, cv::FONT_HERSHEY_SIMPLEX, size, color, 1, cv::LINE_AA);
}
std::string Fixed(double v, int precision = 2) {
    std::ostringstream s;
    s.imbue(std::locale::classic());
    s << std::fixed << std::setprecision(precision) << v;
    return s.str();
}
void Box(cv::Mat &image, const OBB2 &b, const WorldTransform &transform, const cv::Scalar &color) {
    std::vector<cv::Point> polygon;
    for (auto p : Corners(b))
        polygon.push_back(transform.Pixel(p));
    cv::fillConvexPoly(image, polygon, color, cv::LINE_AA);
    cv::polylines(image, polygon, true, ink, 1, cv::LINE_AA);
    const auto forward = b.center + Vec2{std::cos(b.yaw), std::sin(b.yaw)} * (b.length * .4);
    cv::arrowedLine(image, transform.Pixel(b.center), transform.Pixel(forward),
                    cv::Scalar(25, 30, 40), 2, cv::LINE_AA, 0, .35);
}
} // namespace
cv::Point WorldTransform::Pixel(Vec2 p) const {
    RequirePositive(pixelsPerMeter, "pixels per meter");
    return {RoundPixel(left + (p.x - xMin) * pixelsPerMeter),
            RoundPixel(top + (yMax - p.y) * pixelsPerMeter)};
}
Vec2 WorldTransform::World(cv::Point p) const {
    RequirePositive(pixelsPerMeter, "pixels per meter");
    return {xMin + (static_cast<double>(p.x) - left) / pixelsPerMeter,
            yMax - (static_cast<double>(p.y) - top) / pixelsPerMeter};
}
Renderer::Renderer(const ValidationContext &ctx, const Report &r, RenderOptions opts)
    : context_(ctx), report_(r), options_(opts) {
    if (ctx.frames.empty() || opts.width < 640 || opts.height < 480 || opts.width > 8192 ||
        opts.height > 8192)
        throw std::invalid_argument("invalid renderer dimensions/context");
    double xmin = ctx.frames.front().position.x, xmax = xmin, ymin = -r.meta.params.laneHalfWidthM,
           ymax = -ymin;
    auto extend = [&](const OBB2 &box) {
        for (auto p : Corners(box)) {
            xmin = std::min(xmin, p.x);
            xmax = std::max(xmax, p.x);
            ymin = std::min(ymin, p.y);
            ymax = std::max(ymax, p.y);
        }
    };
    for (const auto &f : ctx.frames) {
        extend(EgoAt(f, r.meta.params.egoBox));
        for (const auto &o : ctx.scenario.obstacles.items)
            extend(ObstacleAt(o, f.t - ctx.frames.front().t));
    }
    xmin -= 3;
    xmax += 3;
    ymin -= 3;
    ymax += 3;
    const double xSpan = xmax - xmin, ySpan = ymax - ymin;
    RequirePositive(xSpan, "viewport x span");
    RequirePositive(ySpan, "viewport y span");
    transform_.xMin = xmin;
    transform_.yMax = ymax;
    transform_.pixelsPerMeter = std::min((opts.width - 120) / xSpan, (opts.height - 230) / ySpan);
    RequirePositive(transform_.pixelsPerMeter, "pixels per meter");
    transform_.left = cvRound((opts.width - (xmax - xmin) * transform_.pixelsPerMeter) * .5);
    transform_.top =
        160 + cvRound(((opts.height - 230) - (ymax - ymin) * transform_.pixelsPerMeter) * .5);
}
cv::Mat Renderer::Draw(std::size_t index) const {
    if (index >= context_.frames.size())
        throw std::out_of_range("frame index");
    const auto &f = context_.frames[index];
    const auto &p = report_.meta.params;
    cv::Mat image(options_.height, options_.width, CV_8UC3, cv::Scalar(24, 29, 38));
    Text(image, "AUTONOMY / SCENARIO VALIDATOR", {40, 43}, .9);
    Text(image, "C++17  |  oriented geometry  |  sampled safety diagnostics", {40, 72}, .5, muted);
    Text(image,
         "TIME  " + Fixed(f.t) + " s     SAMPLE  " + std::to_string(index + 1) + " / " +
             std::to_string(context_.frames.size()) + "     SPEED  " +
             (f.velocity ? Fixed(Norm(*f.velocity)) + " m/s" : "unavailable"),
         {40, 111}, .62);
    std::set<std::string> rules;
    bool error = false, warning = false, collision = false, near = false;
    for (const auto &v : report_.violations)
        if (v.sampleIndex && *v.sampleIndex == index) {
            rules.insert(v.rule);
            error |= v.severity == Severity::Error;
            warning |= v.severity == Severity::Warning;
            collision |= v.rule == "collision_obb";
            near |= v.rule == "near_collision";
        }
    std::string status = collision ? "CONTACT"
                         : near    ? "NEAR COLLISION"
                         : error   ? "LANE DEPARTURE"
                         : warning ? "WARNING"
                                   : "CLEAR";
    const auto color = error ? red : warning ? amber : teal;
    Text(image, status, {options_.width - 300, 45}, .8, color);
    cv::line(image, {40, 134}, {options_.width - 40, 134}, cv::Scalar(62, 70, 84), 1);
    const auto &tr = transform_;
    const double right = tr.World({options_.width - 60, 0}).x;
    cv::rectangle(image, tr.Pixel({tr.xMin, p.laneHalfWidthM}),
                  tr.Pixel({right, -p.laneHalfWidthM}), cv::Scalar(47, 53, 63), cv::FILLED);
    // Bound grid work even when a distant obstacle makes the viewport very large.
    const double span = (options_.width - 120) / tr.pixelsPerMeter;
    const double exponent =
        std::max(0.0, std::ceil(std::log10(span / (baseGridSpacingM * maxGridIntervals))));
    const double spacing = baseGridSpacingM * std::pow(10.0, exponent);
    const double firstGridX = std::ceil(tr.xMin / spacing) * spacing;
    for (int tick = 0; tick <= maxGridIntervals; ++tick) {
        const double x = firstGridX + tick * spacing;
        if (x >= right)
            break;
        cv::line(image, tr.Pixel({x, 5}), tr.Pixel({x, -5}), cv::Scalar(43, 50, 62), 1);
        Text(image, Fixed(x, 0) + "m", tr.Pixel({x, -4.2}), .38, muted);
    }
    for (double y : {-p.laneHalfWidthM, p.laneHalfWidthM})
        cv::line(image, tr.Pixel({tr.xMin, y}), tr.Pixel({right, y}), cv::Scalar(170, 175, 185), 2,
                 cv::LINE_AA);
    cv::line(image, tr.Pixel({tr.xMin, 0}), tr.Pixel({right, 0}), cv::Scalar(80, 88, 100), 1,
             cv::LINE_AA);
    for (std::size_t i = 1; i <= index; ++i)
        cv::line(image, tr.Pixel(context_.frames[i - 1].position),
                 tr.Pixel(context_.frames[i].position), teal, 2, cv::LINE_AA);
    for (const auto &o : context_.scenario.obstacles.items) {
        const auto box = ObstacleAt(o, f.t - context_.frames.front().t);
        Box(image, box, tr, Intersects(EgoAt(f, p.egoBox), box) ? red : amber);
        Text(image, o.id, tr.Pixel(box.center) + cv::Point(-12, -18), .42, ink);
    }
    Box(image, EgoAt(f, p.egoBox), tr, color);
    Text(image, "EGO", tr.Pixel(f.position) + cv::Point(-15, 32), .45, color);
    std::string active = "ACTIVE RULES  ";
    for (const auto &rule : rules)
        active += rule + "  ";
    if (rules.empty())
        active += "none";
    Text(image, active, {40, options_.height - 56}, .48, color);
    Text(image,
         "WORLD: x right / y up / meters     TEAL ego + history   |   AMBER obstacle / warning   | "
         "  RED error",
         {40, options_.height - 26}, .44, muted);
    return image;
}
void Renderer::Export(const std::string &directory, const std::string &videoPath,
                      double fps) const {
    RequirePositive(fps, "video fps");
    if (directory.empty())
        throw std::invalid_argument("frames directory is empty");
    std::filesystem::create_directories(directory);
    for (std::size_t i = 0; i < context_.frames.size(); ++i) {
        std::ostringstream name;
        name << "frame_" << std::setw(5) << std::setfill('0') << i << ".png";
        const auto path = std::filesystem::path(directory) / name.str();
        if (!cv::imwrite(path.string(), Draw(i)))
            throw std::runtime_error("cannot write PNG: " + path.string());
    }
    if (!videoPath.empty()) {
        cv::VideoWriter writer;
        for (const int codec : {cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                cv::VideoWriter::fourcc('a', 'v', 'c', '1')}) {
            if (writer.open(videoPath, codec, fps, {options_.width, options_.height}))
                break;
        }
        if (!writer.isOpened())
            throw std::runtime_error(
                "MP4 writer unavailable; PNG frames were exported successfully");
        for (std::size_t i = 0; i < context_.frames.size(); ++i)
            writer.write(Draw(i));
        writer.release();
        if (!std::filesystem::exists(videoPath) || std::filesystem::file_size(videoPath) == 0)
            throw std::runtime_error("video output missing or empty");
    }
}
} // namespace asv::vision
