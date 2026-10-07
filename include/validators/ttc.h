#pragma once
#include "validators/validator.h"
namespace asv {
class TtcValidator final : public IValidator {
  public:
    explicit TtcValidator(double threshold, EgoBox ego = {}) : threshold_(threshold), ego_(ego) {
        RequireNonnegative(threshold, "TTC threshold");
        RequirePositive(ego.length, "ego length");
        RequirePositive(ego.width, "ego width");
    }
    std::string Name() const override { return "ttc"; }
    void Evaluate(const ValidationContext &context, Report &report) const override;

  private:
    double threshold_;
    EgoBox ego_;
};
} // namespace asv
