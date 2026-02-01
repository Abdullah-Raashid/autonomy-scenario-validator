#include "io/scenario_loader.h"
#include "io/csv_reader.h"
#include <cstdlib>

namespace asv
{

    static bool IsHeaderRow(const std::vector<std::string> &row)
    {
        if (row.empty())
            return false;
        // crude check
        return row[0] == "t" || row[0] == "id";
    }

    bool ScenarioLoader::LoadFromCsv(const std::string &trajPath,
                                     const std::string &obsPath,
                                     Scenario &scenario,
                                     std::string &err)
    {
        scenario = Scenario();

        // Load trajectory
        {
            std::vector<std::vector<std::string>> rows;
            if (!CsvReader::ReadAll(trajPath, rows, err))
                return false;

            for (size_t i = 0; i < rows.size(); ++i)
            {
                const std::vector<std::string> &r = rows[i];
                if (r.size() < 3)
                    continue;
                if (IsHeaderRow(r))
                    continue;

                TrajectoryPoint p;
                p.t = std::atof(r[0].c_str());
                p.x = std::atof(r[1].c_str());
                p.y = std::atof(r[2].c_str());
                scenario.ego.points.push_back(p);
            }

            if (scenario.ego.points.size() < 2)
            {
                err = "Trajectory must contain at least 2 points.";
                return false;
            }
        }

        // Load obstacles
        {
            std::vector<std::vector<std::string>> rows;
            if (!CsvReader::ReadAll(obsPath, rows, err))
                return false;

            for (size_t i = 0; i < rows.size(); ++i)
            {
                const std::vector<std::string> &r = rows[i];
                if (r.size() < 5)
                    continue;
                if (IsHeaderRow(r))
                    continue;

                Obstacle o;
                o.id = r[0];
                o.box.cx = std::atof(r[1].c_str());
                o.box.cy = std::atof(r[2].c_str());
                o.box.w = std::atof(r[3].c_str());
                o.box.h = std::atof(r[4].c_str());
                scenario.obstacles.items.push_back(o);
            }
        }

        return true;
    }

} // namespace asv
