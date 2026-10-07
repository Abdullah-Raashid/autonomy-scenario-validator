#include "core/validation.h"
#include "validators/acceleration_limit.h"
#include "validators/collision_obb.h"
#include "validators/jerk_limit.h"
#include "validators/lane_boundary.h"
#include "validators/speed_limit.h"
#include "validators/ttc.h"
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace asv {
void RequireNonnegative(double v, const char *name) {
    if (!std::isfinite(v) || v < 0.0)
        throw std::invalid_argument(std::string(name) + " must be finite and >= 0");
}
void RequirePositive(double v, const char *name) {
    if (!std::isfinite(v) || v <= 0.0)
        throw std::invalid_argument(std::string(name) + " must be finite and > 0");
}
void ValidateParams(const RunParams &p) {
    RequireNonnegative(p.maxSpeedMps, "max speed");
    RequireNonnegative(p.maxAccelMps2, "max acceleration");
    RequireNonnegative(p.maxJerkMps3, "max jerk");
    RequirePositive(p.laneHalfWidthM, "lane half width");
    RequirePositive(p.egoBox.length, "ego length");
    RequirePositive(p.egoBox.width, "ego width");
    RequireNonnegative(p.nearDistanceM, "near distance");
    RequireNonnegative(p.minTtcS, "TTC threshold");
}
namespace {
void Finite(double v) {
    if (!std::isfinite(v))
        throw std::invalid_argument("scenario or derived value is not finite");
}
void Finite(Vec2 v) {
    Finite(v.x);
    Finite(v.y);
}
} // namespace
ValidationContext Prepare(const Scenario &s) {
    if (s.ego.points.empty())
        throw std::invalid_argument("trajectory is empty");
    ValidationContext ctx{s, {}};
    ctx.frames.reserve(s.ego.points.size());
    std::set<std::string> ids;
    for (const auto &o : s.obstacles.items) {
        if (o.id.empty() || !ids.insert(o.id).second)
            throw std::invalid_argument("empty or duplicate obstacle id");
        Finite(o.box.cx);
        Finite(o.box.cy);
        Finite(o.yaw);
        Finite(o.velocity);
        RequirePositive(o.box.w, "obstacle length");
        RequirePositive(o.box.h, "obstacle width");
    }
    // Derivative timestamps are midpoints of the measurements being differenced.
    double previousVelocityTime = 0.0, previousAccelTime = 0.0;
    for (std::size_t i = 0; i < s.ego.points.size(); ++i) {
        const auto &p = s.ego.points[i];
        RequireNonnegative(p.t, "timestamp");
        Finite(p.x);
        Finite(p.y);
        if (p.yaw)
            Finite(*p.yaw);
        FrameState f;
        f.t = p.t;
        f.position = {p.x, p.y};
        f.yaw = p.yaw.value_or(i == 0 ? 0.0 : ctx.frames.back().yaw);
        if (i > 0) {
            const auto &prev = s.ego.points[i - 1];
            const double dt = p.t - prev.t;
            RequirePositive(dt, "timestamp delta");
            f.velocity = (f.position - ctx.frames.back().position) * (1.0 / dt);
            Finite(*f.velocity);
            Finite(Norm(*f.velocity));
            if (!p.yaw && Norm(*f.velocity) > 0.0)
                f.yaw = std::atan2(f.velocity->y, f.velocity->x);
            const double vt = prev.t + dt * 0.5;
            if (i > 1) {
                const double adt = vt - previousVelocityTime;
                RequirePositive(adt, "acceleration delta");
                f.acceleration = (*f.velocity - *ctx.frames.back().velocity) * (1.0 / adt);
                Finite(*f.acceleration);
                Finite(Norm(*f.acceleration));
                const double at = previousVelocityTime + adt * 0.5;
                if (i > 2) {
                    const double jdt = at - previousAccelTime;
                    RequirePositive(jdt, "jerk delta");
                    f.jerk = (*f.acceleration - *ctx.frames.back().acceleration) * (1.0 / jdt);
                    Finite(*f.jerk);
                    Finite(Norm(*f.jerk));
                }
                previousAccelTime = at;
            }
            previousVelocityTime = vt;
        }
        ctx.frames.push_back(f);
    }
    // Initial orientation uses first displacement, while initial velocity remains unknown.
    if (ctx.frames.size() > 1 && !s.ego.points.front().yaw)
        ctx.frames.front().yaw =
            Norm(*ctx.frames[1].velocity) > 0
                ? std::atan2(ctx.frames[1].velocity->y, ctx.frames[1].velocity->x)
                : 0.0;
    for (const auto &f : ctx.frames)
        for (const auto &o : s.obstacles.items)
            Finite(ObstacleAt(o, f.t - ctx.frames.front().t).center);
    return ctx;
}
OBB2 EgoAt(const FrameState &f, EgoBox dims) {
    return {f.position, dims.length, dims.width, f.yaw};
}
OBB2 ObstacleAt(const Obstacle &o, double elapsed) {
    return {{o.box.cx + o.velocity.x * elapsed, o.box.cy + o.velocity.y * elapsed},
            o.box.w,
            o.box.h,
            o.yaw};
}
ValidationEngine::ValidationEngine(RunParams p) : params_(p) {
    ValidateParams(p);
    AddValidator(std::make_unique<SpeedLimitValidator>(p.maxSpeedMps));
    AddValidator(std::make_unique<AccelerationLimitValidator>(p.maxAccelMps2));
    AddValidator(std::make_unique<JerkLimitValidator>(p.maxJerkMps3));
    AddValidator(std::make_unique<LaneBoundaryValidator>(p.laneHalfWidthM, p.egoBox));
    AddValidator(std::make_unique<CollisionObbValidator>(p.egoBox, p.nearDistanceM));
    AddValidator(std::make_unique<TtcValidator>(p.minTtcS, p.egoBox));
}
ValidationEngine::~ValidationEngine() = default;
void ValidationEngine::AddValidator(std::unique_ptr<IValidator> v) {
    if (!v)
        throw std::invalid_argument("validator is null");
    validators_.push_back(std::move(v));
}
Report ValidationEngine::Validate(const Scenario &s) const {
    const auto ctx = Prepare(s);
    Report r;
    r.meta.params = params_;
    r.meta.sampleCount = ctx.frames.size();
    r.meta.scenarioStartT = ctx.frames.front().t;
    r.meta.scenarioEndT = ctx.frames.back().t;
    for (const auto &v : validators_)
        v->Evaluate(ctx, r);
    return r;
}
} // namespace asv
