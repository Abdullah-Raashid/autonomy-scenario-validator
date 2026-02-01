#pragma once
#include "validators/validator.h"

namespace asv
{

    class SpeedLimitValidator : public IValidator
    {
    public:
        explicit SpeedLimitValidator(double maxSpeedMps) : maxSpeedMps_(maxSpeedMps) {}
        std::string Name() const override { return "speed_limit"; }
        void Validate(const Scenario &scenario, Report &report) const override;

    private:
        double maxSpeedMps_;
    };

} // namespace asv
