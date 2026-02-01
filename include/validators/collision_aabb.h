#pragma once
#include "validators/validator.h"
#include "core/types.h"

namespace asv
{

    class CollisionAabbValidator : public IValidator
    {
    public:
        // Ego car box size (w,h)
        CollisionAabbValidator(double egoW, double egoH) : egoW_(egoW), egoH_(egoH) {}
        std::string Name() const override { return "collision_aabb"; }
        void Validate(const Scenario &scenario, Report &report) const override;

    private:
        double egoW_;
        double egoH_;
    };

} // namespace asv
