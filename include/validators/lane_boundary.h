#pragma once
#include "validators/validator.h"

namespace asv
{

    class LaneBoundaryValidator : public IValidator
    {
    public:
        // For MVP: lane corridor is y in [-halfWidth, +halfWidth]
        explicit LaneBoundaryValidator(double halfWidthMeters) : halfWidth_(halfWidthMeters) {}
        std::string Name() const override { return "lane_boundary"; }
        void Validate(const Scenario &scenario, Report &report) const override;

    private:
        double halfWidth_;
    };

} // namespace asv
