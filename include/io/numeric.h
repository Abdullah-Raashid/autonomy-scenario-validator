#pragma once
#include <string>

namespace asv {
// Locale-independent, complete-token, finite decimal parsing; throws invalid_argument.
double ParseFiniteDouble(const std::string &text);
} // namespace asv
