#include "io/csv_reader.h"
#include <fstream>
#include <stdexcept>
#include <utility>

namespace asv {
namespace {
std::string Trim(const std::string &s) {
    const auto begin = s.find_first_not_of(" \t\r");
    if (begin == std::string::npos)
        return {};
    return s.substr(begin, s.find_last_not_of(" \t\r") - begin + 1);
}
std::vector<std::string> Split(const std::string &line) {
    std::vector<std::string> cells;
    std::size_t pos = 0;
    while (true) {
        while (pos < line.size() && (line[pos] == ' ' || line[pos] == '\t'))
            ++pos;
        std::string cell;
        if (pos < line.size() && line[pos] == '"') {
            ++pos;
            bool closed = false;
            while (pos < line.size()) {
                const char c = line[pos++];
                if (c != '"')
                    cell += c;
                else if (pos < line.size() && line[pos] == '"') {
                    cell += '"';
                    ++pos;
                } else {
                    closed = true;
                    break;
                }
            }
            if (!closed)
                throw std::invalid_argument(
                    "unterminated quoted field (multiline fields unsupported)");
            while (pos < line.size() &&
                   (line[pos] == ' ' || line[pos] == '\t' || line[pos] == '\r'))
                ++pos;
            if (pos < line.size() && line[pos] != ',')
                throw std::invalid_argument("characters after quoted field");
        } else {
            while (pos < line.size() && line[pos] != ',') {
                if (line[pos] == '"')
                    throw std::invalid_argument("quote inside unquoted field");
                cell += line[pos++];
            }
            cell = Trim(cell);
        }
        cells.push_back(std::move(cell));
        if (pos == line.size())
            break;
        ++pos; // comma; next iteration preserves a trailing empty field
    }
    return cells;
}
} // namespace
bool CsvReader::ReadAll(const std::string &path, std::vector<std::vector<std::string>> &rows,
                        std::string &err) {
    std::ifstream input(path);
    if (!input) {
        err = "cannot open " + path;
        return false;
    }
    std::vector<std::vector<std::string>> parsed;
    std::string line;
    std::size_t number = 0;
    try {
        while (std::getline(input, line)) {
            ++number;
            if (number == 1 && line.compare(0, 3, "\xef\xbb\xbf") == 0)
                line.erase(0, 3);
            if (Trim(line).empty())
                continue;
            parsed.push_back(Split(line));
        }
        if (input.bad())
            throw std::runtime_error("I/O read failure");
    } catch (const std::exception &e) {
        err = path + ": line " + std::to_string(number) + ": " + e.what();
        return false;
    }
    rows = std::move(parsed);
    err.clear();
    return true;
}
} // namespace asv
