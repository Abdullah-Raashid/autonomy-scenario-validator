#pragma once
#include "validators/validator.h"
namespace asv {
// Reference baseline. Ego extents remain aligned to world x/y regardless of yaw.
class CollisionAabbValidator final : public IValidator {
  public:
    CollisionAabbValidator(double xExtent, double yExtent) : xExtent_(xExtent), yExtent_(yExtent) {
        RequirePositive(xExtent, "x extent");
        RequirePositive(yExtent, "y extent");
    }
    std::string Name() const override { return "collision_aabb"; }
    void Evaluate(const ValidationContext &context, Report &report) const override;

  private:
    double xExtent_, yExtent_;
};
} // namespace asv
