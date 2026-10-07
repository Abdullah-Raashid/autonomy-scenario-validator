#pragma once
#include "validators/validator.h"
namespace asv {
class AccelerationLimitValidator final : public IValidator {
  public:
    explicit AccelerationLimitValidator(double threshold) : threshold_(threshold) {
        RequireNonnegative(threshold, "max acceleration");
    }
    std::string Name() const override { return "acceleration_limit"; }
    void Evaluate(const ValidationContext &context, Report &report) const override;

  private:
    double threshold_;
};
} // namespace asv
