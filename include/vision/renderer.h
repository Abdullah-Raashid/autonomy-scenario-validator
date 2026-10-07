#pragma once
#include "core/validation.h"
#include <opencv2/core.hpp>
#include <string>

namespace asv::vision {
struct WorldTransform {
    double xMin = 0, yMax = 0, pixelsPerMeter = 1;
    int left = 60, top = 150;
    cv::Point Pixel(Vec2 world) const;
    Vec2 World(cv::Point pixel) const;
};
struct RenderOptions {
    int width = 1280, height = 720;
};
class Renderer {
  public:
    Renderer(const ValidationContext &context, const Report &report, RenderOptions options = {});
    cv::Mat Draw(std::size_t index) const;
    void Export(const std::string &directory, const std::string &videoPath = {},
                double fps = 10) const;
    const WorldTransform &Transform() const noexcept { return transform_; }

  private:
    const ValidationContext &context_;
    const Report &report_;
    RenderOptions options_;
    WorldTransform transform_;
};
} // namespace asv::vision
