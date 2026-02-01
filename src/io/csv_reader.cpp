#include "io/csv_reader.h"
#include <fstream>
#include <sstream>

namespace asv
{

    static std::vector<std::string> SplitCsvLine(const std::string &line)
    {
        // Simple CSV: no quoted commas (good enough for MVP)
        std::vector<std::string> out;
        std::string cell;
        std::stringstream ss(line);
        while (std::getline(ss, cell, ','))
        {
            out.push_back(cell);
        }
        return out;
    }

    bool CsvReader::ReadAll(const std::string &path, std::vector<std::vector<std::string>> &rows, std::string &err)
    {
        std::ifstream ifs(path.c_str());
        if (!ifs)
        {
            err = "Failed to open file: " + path;
            return false;
        }
        rows.clear();

        std::string line;
        while (std::getline(ifs, line))
        {
            if (line.empty())
                continue;
            rows.push_back(SplitCsvLine(line));
        }
        return true;
    }

} // namespace asv
