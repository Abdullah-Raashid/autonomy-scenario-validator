#pragma once
#include "validators/validator.h"

namespace asv
{

    class AccelerationLimitValidator : public IValidator
    {
    public:
        explicit AccelerationLimitValidator(double maxAccelMps2) : maxAccelMps2_(maxAccelMps2) {}
        std::string Name() const override { return "acceleration_limit"; }
        void Validate(const Scenario &scenario, Report &report) const override;

    private:
        double maxAccelMps2_;
    };

} // namespace asv
