#pragma once
#include <string>
#include "model/scenario.h"
#include "core/report.h"

namespace asv
{

    class IValidator
    {
    public:
        virtual ~IValidator() {}
        virtual std::string Name() const = 0;
        virtual void Validate(const Scenario &scenario, Report &report) const = 0;
    };

} // namespace asv
