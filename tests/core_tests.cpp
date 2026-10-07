#include "core/validation.h"
#include "validators/acceleration_limit.h"
#include "validators/collision_aabb.h"
#include "validators/collision_obb.h"
#include "validators/jerk_limit.h"
#include "validators/lane_boundary.h"
#include "validators/speed_limit.h"
#include "validators/ttc.h"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <locale>

namespace {
asv::Scenario Straight() {
    asv::Scenario s;
    s.ego.points = {{0, 0, 0, 0}, {1, 2, 0, 0}, {2, 4, 0, 0}, {3, 6, 0, 0}};
    return s;
}
constexpr double pi = 3.14159265358979323846;
} // namespace
TEST(Kinematics, EmptyTrajectoryRejected) {
    EXPECT_THROW(asv::ValidationEngine().Validate({}), std::invalid_argument);
}
TEST(Kinematics, SingleSampleHasNoInventedDerivatives) {
    auto s = Straight();
    s.ego.points.resize(1);
    const auto r = asv::ValidationEngine().Validate(s);
    EXPECT_FALSE(r.metrics.maxSpeedMps);
    EXPECT_FALSE(r.metrics.maxAccelMps2);
    EXPECT_FALSE(r.metrics.maxJerkMps3);
}
TEST(Kinematics, InvalidTimeDeltasRejected) {
    for (double t : {0.0, -1.0}) {
        auto s = Straight();
        s.ego.points[1].t = t;
        EXPECT_THROW(asv::Prepare(s), std::invalid_argument);
    }
}
TEST(Kinematics, NonfiniteInputsAndDerivativesRejected) {
    auto s = Straight();
    s.ego.points[1].x = std::numeric_limits<double>::infinity();
    EXPECT_THROW(asv::Prepare(s), std::invalid_argument);
    s = Straight();
    s.ego.points[1].yaw = std::nan("");
    EXPECT_THROW(asv::Prepare(s), std::invalid_argument);
    s = Straight();
    s.ego.points[0].x = -1e308;
    s.ego.points[1].x = 1e308;
    EXPECT_THROW(asv::Prepare(s), std::invalid_argument);
}
TEST(Kinematics, UnequalSpacingUsesDerivativeMidpoints) {
    asv::Scenario s;
    s.ego.points = {{0, 0, 0, 0}, {1, 1, 0, 0}, {3, 9, 0, 0}, {6, 36, 0, 0}};
    const auto ctx = asv::Prepare(s);
    EXPECT_DOUBLE_EQ(ctx.frames[2].acceleration->x, 2.0);
    EXPECT_DOUBLE_EQ(ctx.frames[3].acceleration->x, 2.0);
    EXPECT_DOUBLE_EQ(ctx.frames[3].jerk->x, 0.0);
}
TEST(Kinematics, AccelerationIncludesChangeInDirection) {
    asv::Scenario s;
    s.ego.points = {{0, 0, 0, {}}, {1, 1, 0, {}}, {2, 1, 1, {}}};
    EXPECT_NEAR(asv::Norm(*asv::Prepare(s).frames[2].acceleration), std::sqrt(2.0), 1e-12);
}
TEST(Kinematics, StationaryYawRetainsLastHeading) {
    asv::Scenario s;
    s.ego.points = {{0, 0, 0, {}}, {1, 0, 2, {}}, {2, 0, 2, {}}};
    const auto ctx = asv::Prepare(s);
    EXPECT_NEAR(ctx.frames[0].yaw, pi / 2, 1e-12);
    EXPECT_NEAR(ctx.frames[2].yaw, pi / 2, 1e-12);
}
TEST(Validators, SpeedEqualityAndThreshold) {
    asv::Report r;
    asv::SpeedLimitValidator(2).Validate(Straight(), r);
    EXPECT_TRUE(r.violations.empty());
    asv::SpeedLimitValidator(1.9).Validate(Straight(), r);
    EXPECT_EQ(r.violations.size(), 3U);
    EXPECT_DOUBLE_EQ(*r.metrics.maxSpeedMps, 2.0);
}
TEST(Validators, AccelerationAndJerk) {
    asv::Scenario s;
    s.ego.points = {{0, 0, 0, 0}, {1, 1, 0, 0}, {2, 3, 0, 0}, {3, 7, 0, 0}};
    asv::Report r;
    asv::AccelerationLimitValidator(1).Validate(s, r);
    ASSERT_EQ(r.violations.size(), 1U);
    EXPECT_DOUBLE_EQ(r.violations.front().t, 3);
    r.violations.clear();
    asv::JerkLimitValidator(1).Validate(s, r);
    EXPECT_TRUE(r.violations.empty());
    asv::JerkLimitValidator(0.9).Validate(s, r);
    EXPECT_EQ(r.violations.size(), 1U);
}
TEST(Validators, DecelerationMagnitudeIsChecked) {
    asv::Scenario s;
    s.ego.points = {{0, 0, 0, 0}, {1, 4, 0, 0}, {2, 5, 0, 0}};
    asv::Report r;
    asv::AccelerationLimitValidator(2).Validate(s, r);
    EXPECT_EQ(r.violations.size(), 1U);
}
TEST(Validators, LaneChecksFullFootprintAndEquality) {
    auto s = Straight();
    for (auto &p : s.ego.points)
        p.y = 0.8;
    asv::Report r;
    asv::LaneBoundaryValidator(1.8).Validate(s, r);
    EXPECT_TRUE(r.violations.empty());
    for (auto &p : s.ego.points)
        p.y = 0.81;
    asv::LaneBoundaryValidator(1.8).Validate(s, r);
    EXPECT_EQ(r.violations.size(), 4U);
}
TEST(Validators, RotatedFootprintCanDepartWithCenterInsideLane) {
    auto s = Straight();
    for (auto &p : s.ego.points)
        p.yaw = pi / 2;
    asv::Report r;
    asv::LaneBoundaryValidator(1.8).Validate(s, r);
    EXPECT_EQ(r.violations.size(), 4U);
    EXPECT_NEAR(*r.metrics.minLaneMarginM, -0.45, 1e-12);
}
TEST(Validators, MissingObstaclesLeaveMetricsUnavailable) {
    const auto r = asv::ValidationEngine().Validate(Straight());
    EXPECT_FALSE(r.metrics.minObstacleDistanceM);
    EXPECT_FALSE(r.metrics.minTtcS);
    EXPECT_EQ(r.ComputeSummary().result, "PASS");
}
TEST(Validators, CollisionAndNearMissAreDistinct) {
    auto s = Straight();
    s.ego.points.resize(1);
    s.obstacles.items = {{"box", {4, 0, 2, 2}, 0, {}}};
    asv::Report r;
    asv::CollisionObbValidator({}, 1).Validate(s, r);
    ASSERT_EQ(r.violations.size(), 1U);
    EXPECT_EQ(r.violations[0].rule, "near_collision");
    EXPECT_DOUBLE_EQ(*r.metrics.minObstacleDistanceM, 0.75);
    s.obstacles.items[0].box.cx = 3.25;
    r = {};
    asv::CollisionObbValidator().Validate(s, r);
    ASSERT_EQ(r.violations.size(), 1U);
    EXPECT_EQ(r.violations[0].severity, asv::Severity::Error);
}
TEST(Validators, DynamicObstacleUsesScenarioStartAsEpoch) {
    auto s = Straight();
    s.ego.points = {{10, 0, 0, 0}, {11, 0, 0, 0}};
    s.obstacles.items = {{"moving", {10, 0, 2, 2}, 0, {-10, 0}}};
    asv::Report r;
    asv::CollisionObbValidator().Validate(s, r);
    ASSERT_EQ(r.violations.size(), 1U);
    EXPECT_DOUBLE_EQ(r.violations[0].t, 11);
}
TEST(Validators, TtcThresholdEqualityAndUnknownInitialVelocity) {
    auto s = Straight();
    s.ego.points.resize(2);
    s.obstacles.items = {{"ahead", {9.25, 0, 2, 2}, 0, {}}};
    asv::Report r;
    asv::TtcValidator(2).Validate(s, r);
    EXPECT_TRUE(r.violations.empty());
    EXPECT_DOUBLE_EQ(*r.metrics.minTtcS, 2);
    asv::TtcValidator(2.1).Validate(s, r);
    ASSERT_EQ(r.violations.size(), 1U);
    EXPECT_EQ(r.violations[0].sampleIndex, 1U);
}
TEST(Validators, InvalidConfigurationRejected) {
    EXPECT_THROW(asv::SpeedLimitValidator(-1), std::invalid_argument);
    EXPECT_THROW(asv::AccelerationLimitValidator(std::nan("")), std::invalid_argument);
    EXPECT_THROW(asv::JerkLimitValidator(-1), std::invalid_argument);
    EXPECT_THROW(asv::LaneBoundaryValidator(0), std::invalid_argument);
    EXPECT_THROW(asv::CollisionObbValidator((asv::EgoBox{0, 1})), std::invalid_argument);
    EXPECT_THROW(asv::TtcValidator(-1), std::invalid_argument);
    asv::RunParams p;
    p.egoBox.width = -1;
    EXPECT_THROW(asv::ValidationEngine{p}, std::invalid_argument);
}
TEST(Geometry, ContactOverlapAndDistance) {
    const asv::OBB2 a{{0, 0}, 2, 2, 0}, touch{{2, 0}, 2, 2, 0}, gap{{3, 4}, 2, 2, 0};
    EXPECT_TRUE(asv::Intersects(a, a));
    EXPECT_TRUE(asv::Intersects(a, touch));
    EXPECT_FALSE(asv::Intersects(a, gap));
    EXPECT_DOUBLE_EQ(asv::BoxDistance(a, touch), 0);
    EXPECT_NEAR(asv::BoxDistance(a, gap), std::sqrt(5), 1e-12);
}
TEST(Geometry, RotatedThinBoxesHaveAabbFalsePositive) {
    const asv::OBB2 a{{0, 0}, 4, 0.2, pi / 4}, b{{0, 1}, 4, 0.2, pi / 4};
    EXPECT_FALSE(asv::Intersects(a, b));
    EXPECT_GT(asv::BoxDistance(a, b), 0);
    EXPECT_TRUE(asv::Intersects(asv::AABB2{0, 0, 3, 3}, asv::AABB2{0, 1, 3, 3}));
}
TEST(Geometry, SymmetryAndTranslationInvariance) {
    for (int i = 0; i < 50; ++i) {
        asv::OBB2 a{{0, 0}, 3, 1, 0.07 * i}, b{{4, 2}, 2, 1, -0.05 * i};
        const double d = asv::BoxDistance(a, b);
        EXPECT_NEAR(d, asv::BoxDistance(b, a), 1e-12);
        a.center = a.center + asv::Vec2{100, -30};
        b.center = b.center + asv::Vec2{100, -30};
        EXPECT_NEAR(d, asv::BoxDistance(a, b), 1e-12);
    }
}
TEST(Geometry, SatAgreesWithAabbForAxisAlignedBoxes) {
    for (int x = -5; x <= 5; ++x)
        for (int y = -5; y <= 5; ++y)
            EXPECT_EQ(
                asv::Intersects(asv::OBB2{{0, 0}, 4, 2, 0},
                                asv::OBB2{{double(x), double(y)}, 2, 2, 0}),
                asv::Intersects(asv::AABB2{0, 0, 4, 2}, asv::AABB2{double(x), double(y), 2, 2}));
}
TEST(Geometry, TtcClosingRecedingParallelAndOverlap) {
    const asv::OBB2 a{{0, 0}, 2, 2, 0}, b{{10, 0}, 2, 2, 0}, c{{10, 4}, 2, 2, 0};
    ASSERT_TRUE(asv::TimeToCollision(a, {2, 0}, b, {}));
    EXPECT_DOUBLE_EQ(*asv::TimeToCollision(a, {2, 0}, b, {}), 4);
    EXPECT_FALSE(asv::TimeToCollision(a, {-2, 0}, b, {}));
    EXPECT_FALSE(asv::TimeToCollision(a, {2, 0}, c, {}));
    EXPECT_FALSE(asv::TimeToCollision(a, {2, 0}, b, {2, 0}));
    EXPECT_DOUBLE_EQ(*asv::TimeToCollision(a, {}, a, {}), 0);
}
TEST(Geometry, TtcCrossingMotionAndFutureContact) {
    const asv::OBB2 a{{0, 0}, 2, 2, 0}, b{{5, 5}, 2, 2, pi / 2};
    auto t = asv::TimeToCollision(a, {1, 0}, b, {0, -1});
    ASSERT_TRUE(t);
    EXPECT_NEAR(*t, 3, 1e-12);
    auto movedA = a, movedB = b;
    movedA.center = movedA.center + asv::Vec2{*t + 1e-8, 0};
    movedB.center = movedB.center + asv::Vec2{0, -*t - 1e-8};
    EXPECT_TRUE(asv::Intersects(movedA, movedB));
}
TEST(Engine, DeterministicAndExtensible) {
    struct Custom final : asv::IValidator {
        std::string Name() const override { return "custom"; }
        void Evaluate(const asv::ValidationContext &c, asv::Report &r) const override {
            r.Add(c.frames[0].t, Name(), asv::Severity::Info, "custom rule", 0);
        }
    };
    asv::ValidationEngine e;
    EXPECT_EQ(e.ValidatorCount(), 6U);
    e.AddValidator(std::make_unique<Custom>());
    const auto r = e.Validate(Straight());
    EXPECT_EQ(r.ToJson(), e.Validate(Straight()).ToJson());
    EXPECT_EQ(r.violations.size(), 1U);
    EXPECT_THROW(e.AddValidator(nullptr), std::invalid_argument);
}
TEST(Engine, DuplicateIdsAndInvalidDimensionsRejected) {
    auto s = Straight();
    s.obstacles.items = {{"dup", {0, 0, 1, 1}, 0, {}}, {"dup", {1, 1, 1, 1}, 0, {}}};
    EXPECT_THROW(asv::Prepare(s), std::invalid_argument);
    s.obstacles.items.resize(1);
    s.obstacles.items[0].box.w = 0;
    EXPECT_THROW(asv::Prepare(s), std::invalid_argument);
}
TEST(Report, SummaryAndDeterministicEscaping) {
    asv::Report r;
    EXPECT_EQ(r.ComputeSummary().result, "PASS");
    EXPECT_FALSE(r.ComputeSummary().firstViolationT);
    r.Add(3, "speed", asv::Severity::Warning, "quote\" slash\\ newline\n control\x01");
    EXPECT_EQ(r.ComputeSummary().result, "WARN");
    r.Add(0, "collision", asv::Severity::Error, "contact");
    const auto s = r.ComputeSummary();
    EXPECT_EQ(s.result, "FAIL");
    EXPECT_DOUBLE_EQ(*s.firstViolationT, 0);
    EXPECT_DOUBLE_EQ(*s.lastViolationT, 3);
    EXPECT_NE(r.ToJson().find("\\u0001"), std::string::npos);
    EXPECT_NE(r.ToJson().find("\\\""), std::string::npos);
    EXPECT_EQ(r.ToJson(), r.ToJson());
    EXPECT_NE(r.ToJson().find("\"min_ttc_s\": null"), std::string::npos);
}
TEST(Report, NonfiniteValuesCannotProduceInvalidJson) {
    asv::Report r;
    EXPECT_THROW(r.Add(std::nan(""), "rule", asv::Severity::Error, ""), std::invalid_argument);
    r.metrics.minTtcS = std::numeric_limits<double>::infinity();
    EXPECT_THROW(r.ToJson(), std::invalid_argument);
}
TEST(Report, LocaleIndependentDecimalNotation) {
    struct Comma : std::numpunct<char> {
        char do_decimal_point() const override { return ','; }
    };
    const auto old = std::locale();
    std::locale::global(std::locale(old, new Comma));
    asv::Report r;
    r.meta.scenarioEndT = 1.25;
    const auto json = r.ToJson();
    std::locale::global(old);
    EXPECT_NE(json.find("1.25"), std::string::npos);
}

TEST(Report, Utf8RoundtripAndMalformedSequencesRejected) {
    asv::Report r;
    r.meta.trajectoryFile = u8"道路.csv";
    EXPECT_NE(r.ToJson().find(u8"道路.csv"), std::string::npos);
    r.meta.trajectoryFile = std::string("\xc0\xaf", 2);
    EXPECT_THROW(r.ToJson(), std::invalid_argument);
    r.meta.trajectoryFile = std::string("\xed\xa0\x80", 3);
    EXPECT_THROW(r.ToJson(), std::invalid_argument);
}
TEST(Geometry, TinyEdgesAndOverflowDoNotProduceNanMetrics) {
    const asv::OBB2 a{{0, 0}, 1e-200, 1e-200, 0}, b{{1, 0}, 1e-200, 1e-200, 0};
    EXPECT_NEAR(asv::BoxDistance(a, b), 1.0, 1e-12);
    const asv::OBB2 huge{{1e308, 0}, 1, 1, 0}, opposite{{-1e308, 0}, 1, 1, 0};
    EXPECT_THROW(asv::BoxDistance(huge, opposite), std::invalid_argument);
    EXPECT_THROW(asv::TimeToCollision(huge, {}, opposite, {}), std::invalid_argument);
}
TEST(Validators, AabbBaselineAgreesWithObbAtZeroYaw) {
    auto s = Straight();
    s.ego.points.resize(1);
    s.obstacles.items = {{"contact", {0, 0, 1, 1}, 0, {}}};
    asv::Report a, b;
    asv::CollisionAabbValidator(4.5, 2).Validate(s, a);
    asv::CollisionObbValidator().Validate(s, b);
    ASSERT_EQ(a.violations.size(), 1U);
    ASSERT_EQ(b.violations.size(), 1U);
    EXPECT_EQ(a.violations[0].t, b.violations[0].t);
}
