#pragma once
#include "core/validation.h"
#include <string>

namespace asv {
class IValidator {
  public:
    virtual ~IValidator() = default;
    virtual std::string Name() const = 0;
    virtual void Evaluate(const ValidationContext &context, Report &report) const = 0;
    // Standalone convenience; the engine prepares kinematics once for all rules.
    void Validate(const Scenario &scenario, Report &report) const {
        Evaluate(Prepare(scenario), report);
    }
};
} // namespace asv
