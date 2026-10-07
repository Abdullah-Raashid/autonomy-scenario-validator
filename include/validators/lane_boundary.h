#pragma once
#include "validators/validator.h"
namespace asv {
class LaneBoundaryValidator final : public IValidator {
  public:
    explicit LaneBoundaryValidator(double halfWidth, EgoBox ego = {})
        : halfWidth_(halfWidth), ego_(ego) {
        RequirePositive(halfWidth, "lane half width");
        RequirePositive(ego.length, "ego length");
        RequirePositive(ego.width, "ego width");
    }
    std::string Name() const override { return "lane_boundary"; }
    void Evaluate(const ValidationContext &context, Report &report) const override;

  private:
    double halfWidth_;
    EgoBox ego_;
};
} // namespace asv
