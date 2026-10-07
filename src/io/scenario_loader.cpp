#include "io/scenario_loader.h"
#include "core/validation.h"
#include "io/csv_reader.h"
#include "io/numeric.h"
#include <stdexcept>
#include <utility>

namespace asv {
namespace {
using Rows = std::vector<std::vector<std::string>>;
Rows Read(const std::string &path) {
    Rows rows;
    std::string error;
    if (!CsvReader::ReadAll(path, rows, error))
        throw std::invalid_argument(error);
    return rows;
}
void Header(const Rows &rows, const std::vector<std::vector<std::string>> &accepted) {
    if (!rows.empty())
        for (const auto &h : accepted)
            if (rows.front() == h)
                return;
    throw std::invalid_argument("missing or unsupported CSV header");
}
} // namespace
bool ScenarioLoader::LoadFromCsv(const std::string &trajPath, const std::string &obsPath,
                                 Scenario &scenario, std::string &err) {
    Scenario parsed;
    std::string path = trajPath;
    std::size_t row = 1;
    try {
        const auto tr = Read(trajPath);
        Header(tr, {{"t", "x", "y"}, {"t", "x", "y", "yaw"}});
        parsed.ego.points.reserve(tr.size() - 1);
        for (std::size_t i = 1; i < tr.size(); ++i) {
            row = i + 1;
            if (tr[i].size() != tr.front().size())
                throw std::invalid_argument("wrong column count");
            TrajectoryPoint p;
            p.t = ParseFiniteDouble(tr[i][0]);
            p.x = ParseFiniteDouble(tr[i][1]);
            p.y = ParseFiniteDouble(tr[i][2]);
            if (tr[i].size() == 4)
                p.yaw = ParseFiniteDouble(tr[i][3]);
            RequireNonnegative(p.t, "timestamp");
            if (!parsed.ego.points.empty() && p.t <= parsed.ego.points.back().t)
                throw std::invalid_argument("timestamps must strictly increase");
            parsed.ego.points.push_back(p);
        }
        if (parsed.ego.points.size() < 2)
            throw std::invalid_argument("trajectory requires at least two samples");
        path = obsPath;
        row = 1;
        if (!obsPath.empty()) {
            const auto ob = Read(obsPath);
            Header(ob,
                   {{"id", "cx", "cy", "w", "h"}, {"id", "cx", "cy", "w", "h", "yaw", "vx", "vy"}});
            parsed.obstacles.items.reserve(ob.size() - 1);
            for (std::size_t i = 1; i < ob.size(); ++i) {
                row = i + 1;
                if (ob[i].size() != ob.front().size())
                    throw std::invalid_argument("wrong column count");
                Obstacle o;
                o.id = ob[i][0];
                o.box = {ParseFiniteDouble(ob[i][1]), ParseFiniteDouble(ob[i][2]),
                         ParseFiniteDouble(ob[i][3]), ParseFiniteDouble(ob[i][4])};
                RequirePositive(o.box.w, "obstacle length");
                RequirePositive(o.box.h, "obstacle width");
                if (ob[i].size() == 8) {
                    o.yaw = ParseFiniteDouble(ob[i][5]);
                    o.velocity = {ParseFiniteDouble(ob[i][6]), ParseFiniteDouble(ob[i][7])};
                }
                parsed.obstacles.items.push_back(std::move(o));
            }
        }
        path = "scenario consistency";
        row = 0;
        Prepare(parsed);
        scenario = std::move(parsed);
        err.clear();
        return true;
    } catch (const std::exception &e) {
        err = path + (row == 0 ? ": " : ": CSV row " + std::to_string(row) + ": ") + e.what();
        return false; // leave caller's scenario untouched on failure
    }
}
} // namespace asv
