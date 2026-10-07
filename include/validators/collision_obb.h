#pragma once
#include "validators/validator.h"
namespace asv {
class CollisionObbValidator final : public IValidator {
  public:
    explicit CollisionObbValidator(EgoBox ego = {}, double nearDistance = 1.0)
        : ego_(ego), nearDistance_(nearDistance) {
        RequirePositive(ego.length, "ego length");
        RequirePositive(ego.width, "ego width");
        RequireNonnegative(nearDistance, "near distance");
    }
    std::string Name() const override { return "collision_obb"; }
    void Evaluate(const ValidationContext &context, Report &report) const override;

  private:
    EgoBox ego_;
    double nearDistance_;
};
} // namespace asv
