#include "vision/lane_detector.h"
#include "vision/renderer.h"
#include <chrono>
#include <filesystem>
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

TEST(SyntheticVision, BothLanesRecoverKnownBottomCoordinates) {
    const auto image = asv::vision::SyntheticRoad();
    const auto r = asv::vision::DetectLanes(image);
    ASSERT_TRUE(r.left);
    ASSERT_TRUE(r.right);
    EXPECT_NEAR(r.left->X(539), .20 * 959, 12);
    EXPECT_NEAR(r.right->X(539), .80 * 959, 12);
    EXPECT_LT(r.left->a, 0);
    EXPECT_GT(r.right->a, 0);
    EXPECT_EQ(r.gray.type(), CV_8UC1);
    EXPECT_EQ(r.overlay.type(), CV_8UC3);
}
TEST(SyntheticVision, DeterminismAndInputOwnership) {
    auto image = asv::vision::SyntheticRoad();
    const auto before = image.clone();
    const auto a = asv::vision::DetectLanes(image), b = asv::vision::DetectLanes(image);
    EXPECT_EQ(cv::norm(image, before, cv::NORM_INF), 0);
    EXPECT_EQ(cv::norm(image, asv::vision::SyntheticRoad(), cv::NORM_INF), 0);
    EXPECT_EQ(cv::norm(a.roiEdges, b.roiEdges, cv::NORM_INF), 0);
    EXPECT_EQ(a.candidates, b.candidates);
    ASSERT_TRUE(a.left);
    ASSERT_TRUE(b.left);
    EXPECT_DOUBLE_EQ(a.left->a, b.left->a);
}
TEST(SyntheticVision, BlankImageDoesNotInventLanes) {
    const auto r = asv::vision::DetectLanes(cv::Mat::zeros(540, 960, CV_8UC3));
    EXPECT_FALSE(r.left);
    EXPECT_FALSE(r.right);
    EXPECT_EQ(cv::countNonZero(r.roiEdges), 0);
}
TEST(SyntheticVision, HorizontalDistractorsRejected) {
    cv::Mat image = cv::Mat::zeros(540, 960, CV_8UC1);
    cv::line(image, {100, 450}, {860, 450}, 255, 5);
    const auto r = asv::vision::DetectLanes(image);
    EXPECT_FALSE(r.left);
    EXPECT_FALSE(r.right);
}
TEST(SyntheticVision, SingleLaneLeavesOtherSideUnavailable) {
    auto image = asv::vision::SyntheticRoad();
    cv::rectangle(image, {480, 0, 480, 540}, cv::Scalar(66, 65, 62), cv::FILLED);
    const auto r = asv::vision::DetectLanes(image);
    EXPECT_TRUE(r.left);
    EXPECT_FALSE(r.right);
}
TEST(SyntheticVision, RoiMasksOutsideArea) {
    const auto r = asv::vision::DetectLanes(asv::vision::SyntheticRoad());
    EXPECT_EQ(cv::countNonZero(r.roiEdges(cv::Rect(0, 0, 960, 280))), 0);
    EXPECT_GT(cv::countNonZero(r.roiEdges), 0);
}
TEST(VisionInput, InvalidImageAndConfigRejected) {
    EXPECT_THROW(asv::vision::DetectLanes({}), std::invalid_argument);
    EXPECT_THROW(asv::vision::DetectLanes(cv::Mat(100, 100, CV_32FC3)), std::invalid_argument);
    auto c = asv::vision::LaneConfig{};
    c.roiTop = 1;
    EXPECT_THROW(asv::vision::DetectLanes(asv::vision::SyntheticRoad(), c), std::invalid_argument);
    c = {};
    c.cannyHigh = c.cannyLow;
    EXPECT_THROW(asv::vision::DetectLanes(asv::vision::SyntheticRoad(), c), std::invalid_argument);
}
TEST(SyntheticVision, ResolutionScalingAndDebugPanel) {
    const auto image = asv::vision::SyntheticRoad(640, 360);
    const auto r = asv::vision::DetectLanes(image);
    ASSERT_TRUE(r.left);
    ASSERT_TRUE(r.right);
    EXPECT_NEAR(r.left->X(359), .2 * 639, 12);
    EXPECT_EQ(asv::vision::DebugPanel(image, r).size(), cv::Size(960, 900));
}
TEST(Renderer, WorldPixelRoundtripAndAxisConventions) {
    const asv::vision::WorldTransform t{-5, 10, 20, 60, 160};
    const auto p = t.Pixel({1, 2});
    EXPECT_EQ(p, cv::Point(180, 320));
    const auto world = t.World(p);
    EXPECT_DOUBLE_EQ(world.x, 1);
    EXPECT_DOUBLE_EQ(world.y, 2);
    EXPECT_LT(t.Pixel({0, 3}).y, t.Pixel({0, 2}).y);
}
TEST(Renderer, PngExportRoundtripAndFrameBounds) {
    asv::Scenario s;
    s.ego.points = {{0, 0, 0, 0}, {1, 2, 0, 0}};
    const auto ctx = asv::Prepare(s);
    const auto report = asv::ValidationEngine().Validate(s);
    const asv::vision::Renderer renderer(ctx, report);
    EXPECT_THROW(renderer.Draw(2), std::out_of_range);
    const auto dir = std::filesystem::temp_directory_path() /
                     ("asv-frames-" +
                      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    renderer.Export(dir.string());
    const auto png = cv::imread((dir / "frame_00001.png").string());
    EXPECT_EQ(png.size(), cv::Size(1280, 720));
    EXPECT_EQ(cv::norm(png, renderer.Draw(1), cv::NORM_INF), 0);
    EXPECT_THROW(renderer.Export(dir.string(), "", 0), std::invalid_argument);
    std::filesystem::remove_all(dir);
}

TEST(Renderer, OptionalMp4ExportDecodesWhenCodecAvailable) {
    asv::Scenario s;
    s.ego.points = {{0, 0, 0, 0}, {1, 2, 0, 0}};
    const auto ctx = asv::Prepare(s);
    const auto report = asv::ValidationEngine().Validate(s);
    const auto dir = std::filesystem::temp_directory_path() /
                     ("asv-video-" +
                      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto video = dir / "scenario.mp4";
    try {
        asv::vision::Renderer(ctx, report).Export(dir.string(), video.string(), 2);
    } catch (const std::runtime_error &e) {
        const std::string error = e.what();
        if (error.find("MP4 writer unavailable") == std::string::npos) {
            std::filesystem::remove_all(dir);
            throw;
        }
        EXPECT_TRUE(std::filesystem::exists(dir / "frame_00001.png"));
        std::filesystem::remove_all(dir);
        GTEST_SKIP() << error;
    }
    cv::VideoCapture reader(video.string());
    EXPECT_TRUE(reader.isOpened());
    cv::Mat frame;
    int count = 0;
    while (reader.read(frame)) {
        EXPECT_EQ(frame.size(), cv::Size(1280, 720));
        ++count;
    }
    EXPECT_EQ(count, 2);
    reader.release();
    std::filesystem::remove_all(dir);
}

TEST(Renderer, LargeViewportUsesBoundedGrid) {
    asv::Scenario s;
    s.ego.points = {{0, 0, 0, 0}, {1, 1, 0, 0}};
    s.obstacles.items = {{"distant", {1e9, 0, 2, 2}, 0, {}}};
    const auto ctx = asv::Prepare(s);
    const auto r = asv::ValidationEngine().Validate(s);
    EXPECT_EQ(asv::vision::Renderer(ctx, r).Draw(0).size(), cv::Size(1280, 720));
}
TEST(Renderer, UnrepresentableViewportAndPixelCoordinatesRejected) {
    asv::Scenario s;
    s.ego.points = {{0, 1e308, 0, 0}};
    const auto ctx = asv::Prepare(s);
    const auto r = asv::ValidationEngine().Validate(s);
    EXPECT_THROW(asv::vision::Renderer(ctx, r), std::invalid_argument);
    const asv::vision::WorldTransform transform;
    EXPECT_THROW(transform.Pixel({1e308, 0}), std::invalid_argument);
}
