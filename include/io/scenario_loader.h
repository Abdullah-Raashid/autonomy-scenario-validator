#pragma once
#include <string>
#include "model/scenario.h"

namespace asv
{

    class ScenarioLoader
    {
    public:
        // CSV formats:
        // trajectory: t,x,y
        // obstacles: id,cx,cy,w,h
        static bool LoadFromCsv(const std::string &trajPath,
                                const std::string &obsPath,
                                Scenario &scenario,
                                std::string &err);
    };

} // namespace asv
