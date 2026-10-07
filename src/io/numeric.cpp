#include "io/numeric.h"
#include <cmath>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace asv {
double ParseFiniteDouble(const std::string &text) {
    if (text.find_first_not_of("0123456789+-.eE \t\r\n") != std::string::npos)
        throw std::invalid_argument("expected decimal number: '" + text + "'");
    std::istringstream input(text);
    input.imbue(std::locale::classic());
    double value = 0.0;
    if (!(input >> value) || !std::isfinite(value))
        throw std::invalid_argument("expected finite number: '" + text + "'");
    input >> std::ws;
    if (!input.eof())
        throw std::invalid_argument("invalid numeric suffix: '" + text + "'");
    return value;
}
} // namespace asv
