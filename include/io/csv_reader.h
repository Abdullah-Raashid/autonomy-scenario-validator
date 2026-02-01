#pragma once
#include <string>
#include <vector>

namespace asv
{

    class CsvReader
    {
    public:
        static bool ReadAll(const std::string &path, std::vector<std::vector<std::string>> &rows, std::string &err);
    };

} // namespace asv
