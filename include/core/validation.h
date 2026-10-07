#pragma once
#include "core/report.h"
#include "geometry/obb.h"
#include "model/scenario.h"
#include <memory>
#include <optional>
#include <vector>

namespace asv {
struct FrameState {
    double t = 0.0;
    Vec2 position;
    double yaw = 0.0;
    std::optional<Vec2> velocity, acceleration, jerk;
};
struct ValidationContext {
    const Scenario &scenario; // borrowed for the duration of validation
    std::vector<FrameState> frames;
};
void RequireNonnegative(double value, const char *name);
void RequirePositive(double value, const char *name);
void ValidateParams(const RunParams &params);
ValidationContext Prepare(const Scenario &scenario);
OBB2 EgoAt(const FrameState &frame, EgoBox dimensions);
OBB2 ObstacleAt(const Obstacle &obstacle, double elapsed);
class IValidator;
class ValidationEngine {
  public:
    explicit ValidationEngine(RunParams params = {});
    ~ValidationEngine();
    void AddValidator(std::unique_ptr<IValidator> validator);
    Report Validate(const Scenario &scenario) const;
    std::size_t ValidatorCount() const noexcept { return validators_.size(); }

  private:
    RunParams params_;
    std::vector<std::unique_ptr<IValidator>> validators_;
};
} // namespace asv
