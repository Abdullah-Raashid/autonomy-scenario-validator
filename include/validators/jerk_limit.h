#pragma once
#include "validators/validator.h"
namespace asv {
class JerkLimitValidator final : public IValidator {
  public:
    explicit JerkLimitValidator(double threshold) : threshold_(threshold) {
        RequireNonnegative(threshold, "max jerk");
    }
    std::string Name() const override { return "jerk_limit"; }
    void Evaluate(const ValidationContext &context, Report &report) const override;

  private:
    double threshold_;
};
} // namespace asv
