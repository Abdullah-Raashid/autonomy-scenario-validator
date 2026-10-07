#pragma once
#include "model/scenario.h"
#include <string>
namespace asv {
class ScenarioLoader {
  public:
    // Strict schemas; blank lines ignored; obsPath empty means no obstacles.
    // On failure, scenario remains unchanged and err contains diagnostic context.
    static bool LoadFromCsv(const std::string &trajPath, const std::string &obsPath,
                            Scenario &scenario, std::string &err);
};
} // namespace asv
