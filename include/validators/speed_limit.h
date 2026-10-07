#pragma once
#include "validators/validator.h"
namespace asv {
class SpeedLimitValidator final : public IValidator {
  public:
    explicit SpeedLimitValidator(double threshold) : threshold_(threshold) {
        RequireNonnegative(threshold, "max speed");
    }
    std::string Name() const override { return "speed_limit"; }
    void Evaluate(const ValidationContext &context, Report &report) const override;

  private:
    double threshold_;
};
} // namespace asv
